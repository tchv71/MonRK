/*
 * Wiznet.
 * (c) Copyright 2002, Wiznet.
 *
 * Filename	: ftpd.c
 * Version	: 1.0
 * Programmer(s)	:
 * Created	: 2003/01/28
 * Description   : FTP daemon. (AVR-GCC Compiler)
 */

#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <limits.h>
#include <stdarg.h>
#include <stdlib.h>
//#include "socket.h"
//#include "stdint.h"
#include "ftpd.h"
#include <sys/_stdint.h>
#undef ERR_OK
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "lwip/arch.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"

#define FTP_PORT 21
#define DEBUG_printf printf
#define BUF_SIZE 2048
#define TEST_ITERATIONS 10
#define POLL_TIME_S 5

static TCP_SERVER_T *tcp_ftp_server_init(void)
{
	TCP_SERVER_T *state = calloc(1, sizeof(TCP_SERVER_T));
	if (!state)
	{
		DEBUG_printf("failed to allocate state\n");
		return NULL;
	}
	state->sock_state = SOCK_INIT;
    return state;
}

extern struct ftpd ftp;

static err_t tcp_ftp_server_close(void *arg)
{
    TCP_SERVER_T *state = (TCP_SERVER_T *)arg;
	err_t err = ERR_OK;
    if (state->client_pcb != NULL)
    {
        tcp_arg(state->client_pcb, NULL);
        tcp_poll(state->client_pcb, NULL, 0);
        tcp_sent(state->client_pcb, NULL);
        tcp_recv(state->client_pcb, NULL);
        tcp_err(state->client_pcb, NULL);
        err = tcp_close(state->client_pcb);
        if (err != ERR_OK)
        {
            DEBUG_printf("close failed %d, calling abort\n", err);
            tcp_abort(state->client_pcb);
            err = ERR_ABRT;
        }
        state->client_pcb = NULL;
    }
    if (state->server_pcb)
    {
        tcp_arg(state->server_pcb, NULL);
        tcp_close(state->server_pcb);
        state->server_pcb = NULL;
    }
	state->sock_state = SOCK_CLOSED;
    return err;
}

static err_t tcp_ftp_server_result(void *arg, int status)
{
    TCP_SERVER_T *state = (TCP_SERVER_T *)arg;
    if (status == 0)
    {
        DEBUG_printf("test success\n");
    }
    else
    {
        DEBUG_printf("test failed %d\n", status);
    }
    state->complete = true;
    return tcp_ftp_server_close(arg);
}

static err_t tcp_ftp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
    TCP_SERVER_T *state = (TCP_SERVER_T *)arg;
    DEBUG_printf("tcp_ftp_server_sent %u\n", len);
    state->sent_len += len;

    return ERR_OK;
}

static err_t tcp_server_data_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
    TCP_SERVER_T *state = (TCP_SERVER_T *)arg;
    DEBUG_printf("tcp_server_data_sent %u\n", len);
    state->sent_len += len;

    return ERR_OK;
}


uint8_t ftp_rcv_buf[512]; 
int ftp_rcv_len = 0;
uint8_t ftp_rcv_data_buf[2048]; 
int ftp_rcv_data_len = 0;

struct ftpd ftp;

err_t tcp_ftp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    TCP_SERVER_T *state = (TCP_SERVER_T *)arg;
    if (!p)
    {
        return tcp_ftp_server_result(arg, -1);
    }
    // this method is callback from lwIP, so cyw43_arch_lwip_begin is not required, however you
    // can use this method to cause an assertion in debug mode, if this method is called when
    // cyw43_arch_lwip_begin IS needed
    cyw43_arch_lwip_begin();
    if (p->tot_len > 0)
    {
        u16_t len = p->tot_len;
		if (state == ftp.tcp_state)
		{
			pbuf_copy_partial(p, ftp_rcv_buf, len, 0);
			ftp_rcv_len = len;
		}
		else
		{
			pbuf_copy_partial(p, ftp_rcv_data_buf, len, 0);
			ftp_rcv_data_len = len;
		}
        DEBUG_printf("tcp_ftp_server_recv %d/%d err %d\n", p->tot_len, state->recv_len, err);
        tcp_recved(tpcb, p->tot_len);
    }
    pbuf_free(p);
	cyw43_arch_lwip_end();
    return ERR_OK;
}

err_t tcp_ftp_server_data_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    TCP_SERVER_T *state = (TCP_SERVER_T *)arg;
    if (!p)
    {
        return tcp_ftp_server_result(arg, -1);
    }
    // this method is callback from lwIP, so cyw43_arch_lwip_begin is not required, however you
    // can use this method to cause an assertion in debug mode, if this method is called when
    // cyw43_arch_lwip_begin IS needed
    cyw43_arch_lwip_begin();
    if (p->tot_len > 0)
    {
        u16_t len = p->tot_len;
		pbuf_copy_partial(p, ftp_rcv_data_buf, len, 0);
		ftp_rcv_data_len = len;
        DEBUG_printf("tcp_ftp_server_data_recv %d/%d err %d\n", p->tot_len, state->recv_len, err);
        tcp_recved(tpcb, p->tot_len);
    }
    pbuf_free(p);
	cyw43_arch_lwip_end();
    return ERR_OK;
}

static err_t tcp_ftp_server_poll(void *arg, struct tcp_pcb *tpcb)
{
    DEBUG_printf("tcp_server_poll_fn\n");
    return tcp_ftp_server_result(arg, 0/*-1*/); // no response is an error?
	//return 0;
}

static void tcp_ftp_server_err(void *arg, err_t err)
{
    if (err != ERR_ABRT)
    {
        DEBUG_printf("tcp_client_err_fn %d\n", err);
        tcp_ftp_server_result(arg, err);
    }
}

static err_t tcp_ftp_server_accept(void *arg, struct tcp_pcb *client_pcb, err_t err)
{
    TCP_SERVER_T *state = (TCP_SERVER_T *)arg;
    if (err != ERR_OK || client_pcb == NULL)
    {
        DEBUG_printf("Failure in accept\n");
        tcp_ftp_server_result(arg, err);
        return ERR_VAL;
    }
    DEBUG_printf("Client connected\n");

    state->client_pcb = client_pcb;
    tcp_arg(client_pcb, state);
    tcp_sent(client_pcb, tcp_ftp_server_sent);
    tcp_recv(client_pcb, tcp_ftp_server_recv);
    //tcp_poll(client_pcb, tcp_ftp_server_poll, POLL_TIME_S * 2);
    tcp_err(client_pcb, tcp_ftp_server_err);
	state->sock_state = SOCK_ESTABLISHED;
    return ERR_OK;
}

static bool tcp_ftp_server_open(void *arg, uint16_t port)
{
    TCP_SERVER_T *state = (TCP_SERVER_T *)arg;
    //DEBUG_printf("Starting server at %s on port %u\n", ip4addr_ntoa(netif_ip4_addr(netif_list)), FTP_PORT);

    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_V4);
    if (!pcb)
    {
        DEBUG_printf("failed to create pcb\n");
        return false;
    }
	state->client_pcb = pcb;

	err_t err = tcp_bind(pcb, NULL, port);
    if (err)
    {
        DEBUG_printf("failed to bind to port %u\n", port);
        return false;
    }
	return true;
}

static err_t tcp_ftp_server_data_connected(void *arg, struct tcp_pcb *tpcb, err_t err)
{
    TCP_SERVER_T *state = (TCP_SERVER_T *)arg;
	if (err != ERR_OK)
	{
		printf("connect failed %d\n", err);
		return tcp_ftp_server_result(arg, err);
	}
	state->sock_state = SOCK_ESTABLISHED;
	DEBUG_printf("Waiting for buffer from server\n");
	return ERR_OK;
}

uint16_t remote_port;

static bool tcp_ftp_server_data_open(void *arg)
{
    TCP_SERVER_T *state = (TCP_SERVER_T *)arg;
    DEBUG_printf("Connecting to %s port %u\n", ip4addr_ntoa(&state->remote_addr), FTP_PORT);
    state->client_pcb = tcp_new_ip_type(IP_GET_TYPE(&state->remote_addr));
    if (!state->client_pcb) {
        DEBUG_printf("failed to create pcb\n");
        return false;
    }

    tcp_arg(state->client_pcb, state);
    //tcp_poll(state->client_pcb, tcp_ftp_server_poll, POLL_TIME_S * 2);
    tcp_sent(state->client_pcb, tcp_server_data_sent);
    tcp_recv(state->client_pcb, tcp_ftp_server_data_recv);
    tcp_err(state->client_pcb, tcp_ftp_server_err);

    //state->buffer_len = 0;

    // cyw43_arch_lwip_begin/end should be used around calls into lwIP to ensure correct locking.
    // You can omit them if you are in a callback from lwIP. Note that when using pico_cyw_arch_poll
    // these calls are a no-op and can be omitted, but it is a good practice to use them in
    // case you switch the cyw43_arch type later.
    cyw43_arch_lwip_begin();
    err_t err = tcp_connect(state->client_pcb, &state->remote_addr, state->remote_port, tcp_ftp_server_data_connected);
    cyw43_arch_lwip_end();
	state->sock_state = SOCK_SYNSENT;

    return err == ERR_OK;
}

static bool tcp_ftp_server_listen(void *arg)
{
    TCP_SERVER_T *state = (TCP_SERVER_T *)arg;

    state->server_pcb = tcp_listen_with_backlog(state->client_pcb, 1);
    if (!state->server_pcb)
    {
        DEBUG_printf("failed to listen\n");
        if (state->client_pcb)
        {
            tcp_close(state->client_pcb);
        }
        return false;
    }

    tcp_arg(state->server_pcb, state);
    tcp_accept(state->server_pcb, tcp_ftp_server_accept);
	state->sock_state = SOCK_LISTEN;

    return true;
}


/* Command table */
char *commands[] = {
	"user",
	"acct",
	"pass",
	"type",
	"list",
	"cwd",
	"dele",
	"name",
	"quit",
	"retr",
	"stor",
	"port",
	"nlst",
	"pwd",
	"xpwd",
	"mkd",
	"xmkd",
	"xrmd",
	"rmd ",
	"stru",
	"mode",
	"syst",
	"xmd5",
	"xcwd",
	"feat",
	"pasv",
	"size",
	"mlsd",
	"appe",
	"rnfr",
	"rnto",
	NULL};

#if 0
/* Response messages */
char banner[] = "220 %s FTP version %s ready.\r\n";
char badcmd[] = "500 Unknown command '%s'\r\n";
char binwarn[] = "100 Warning: type is ASCII and %s appears to be binary\r\n";
char unsupp[] = "500 Unsupported command or option\r\n";
char givepass[] = "331 Enter PASS command\r\n";
char logged[] = "230 Logged in\r\n";
char typeok[] = "200 Type %s OK\r\n";
char only8[] = "501 Only logical bytesize 8 supported\r\n";
char deleok[] = "250 File deleted\r\n";
char mkdok[] = "200 MKD ok\r\n";
char delefail[] = "550 Delete failed: %s\r\n";
char pwdmsg[] = "257 \"%s\" is current directory\r\n";
char badtype[] = "501 Unknown type \"%s\"\r\n";
char badport[] = "501 Bad port syntax\r\n";
char unimp[] = "502 Command does not implemented yet.\r\n";
char bye[] = "221 Goodbye!\r\n";
char nodir[] = "553 Can't read directory \"%s\": %s\r\n";
char cantopen[] = "550 Can't read file \"%s\": %s\r\n";
char sending[] = "150 Opening data connection for %s (%d.%d.%d.%d,%d)\r\n";
char cantmake[] = "553 Can't create \"%s\": %s\r\n";
char writerr[] = "552 Write error: %s\r\n";
char portok[] = "200 PORT command successful.\r\n";
char rxok[] = "226 Transfer complete.\r\n";
char txok[] = "226 Transfer complete.\r\n";
char noperm[] = "550 Permission denied\r\n";
char noconn[] = "425 Data connection reset\r\n";
char lowmem[] = "421 System overloaded, try again later\r\n";
char notlog[] = "530 Please log in with USER and PASS\r\n";
char userfirst[] = "503 Login with USER first.\r\n";
char okay[] = "200 Ok\r\n";
char syst[] = "215 %s Type: L%d Version: %s\r\n";
char sizefail[] = "550 File not found\r\n";
#endif

un_l2cval remote_ip;
un_l2cval local_ip;
uint16_t local_port = 35000;
uint16_t local_port_actual = 35000;
uint8_t connect_state_control = 0;
uint8_t connect_state_data = 0;

int current_year = 2014;
int current_month = 12;
int current_day = 31;
int current_hour = 10;
int current_min = 10;
int current_sec = 30;


void ftpd_init(uint8_t *src_ip)
{
	ftp.state = FTPS_NOT_LOGIN;
	ftp.current_cmd = NO_CMD;
	ftp.dsock_mode = ACTIVE_MODE;

	local_ip.cVal[0] = src_ip[0];
	local_ip.cVal[1] = src_ip[1];
	local_ip.cVal[2] = src_ip[2];
	local_ip.cVal[3] = src_ip[3];
	local_port = 35000;

	strcpy(ftp.workingdir, "/");

	ftp.tcp_state = tcp_ftp_server_init();
	//socket(CTRL_SOCK, Sn_MR_TCP, IPPORT_FTP, 0x0);
}

typedef int FRESULT;
#define FR_OK 0
void strip_spaces(char *str, char *right)
{
	*right-- = 0;
	while (right >= str)
	{
		if (*right == ' ')
			*right = 0;
		--right;
	}
}

void get_file_name(char *fname)
{
	memcpy(fname, FS_DIRENTRY + DIR_Name, 8);
	strip_spaces(fname, fname + 8);
	if (strncmp(FS_DIRENTRY + DIR_Name + 8, "   ", 3) != 0)
	{
		strcat(fname, ".");
		size_t l = strlen(fname);
		memcpy(fname + l, FS_DIRENTRY + DIR_Name + 8, 3);
		strip_spaces(fname + l, fname + l + 3);
	}
}

FRESULT scan_files(char *path, char *buf1, int *buf_len)
{
	unsigned int file_cnt = 0;
	FRESULT res;
	FILINFO fno;
	_DIR dir;
	int i, len;
	char *fn;
	char temp_mon[12][4] = {"JAN", "FEB", "MAR", "APR", "MAY", "JUN", "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"};
	char *p_buf = buf1;
	char temp_dir = 0;
	WORD temp_f_date = 0;
	WORD temp_f_time = 0;

#if _USE_LFN
    static char lfn[_MAX_LFN * (_DF1S ? 2 : 1) + 1];
    fno.altname = lfn;
    fno.lfsize = sizeof(lfn);
#endif

	//printf("Open..");
	strcpy(buf, path);
	if (strcmp(buf, "/") == 0)
		buf[0] = 0;
	res = fs_opendir(); // f_opendir(&dir, path);

	if (res == FR_OK)
	{
		i = strlen(path);
		//printf("OK.");
		for (;;)
		{
			res = fs_readdir(); // f_readdir(&dir, &fno);
			if (res != FR_OK || FS_DIRENTRY[0] == 0)
				break;
			if (FS_DIRENTRY[0] == '.')
				continue;
			get_file_name(fno.fname);
			// memcpy(fno.fname, FS_DIRENTRY + DIR_Name, 8);
			// fno.fname[8] = strncmp(FS_DIRENTRY + DIR_Name+8, "   ", 3)!=0 ? '.' : ' ';
			// memcpy(fno.fname+9, FS_DIRENTRY + DIR_Name+8, 3);
			// fno.fname[12] = 0;
			memcpy(&fno.fsize, FS_DIRENTRY + DIR_FileSize, 4);
			memcpy(&fno.ftime, FS_DIRENTRY + DIR_WrtTime, 2);
			memcpy(&fno.fdate, FS_DIRENTRY + DIR_WrtTime + 2, 2);

#if _USE_LFN
            fn = *fno.altname ? fno.altname : fno.fname;
#else
			fn = fno.fname;
#endif
			if (fno.fdate == 0)
			{
				fno.fdate = temp_f_date;
				fno.ftime = temp_f_time;
			}
			else
			{
				temp_f_date = fno.fdate;
				temp_f_time = fno.ftime;
			}
			// get_fileinfo(&dir, &fno);
			//printf("f_readdir ret : %d, fname %c\r\n", res, fno.fname[0]);
			//printf("modtime = %02X%02X%02X%02X \r\n", (dir.dir + 22)[3], (dir.dir + 22)[2], (dir.dir + 22)[1],(dir.dir + 22)[0]);
			//printf("f_date:%x, f_time:%x\r\n", fno.fdate, fno.ftime);

			if (FS_DIRENTRY[DIR_Attr] & AM_DIR)
			{
				temp_dir = 'd';
				//printf("[D]%s\r\n",fn);
			}
			else
			{
				temp_dir = '-';
				//printf("%s/%s : \t\t %dbyte\r\n", path, fn,fno.fsize);
			}
			uint8_t h = fno.ftime >> 11;
			uint8_t m = (fno.ftime >> 5) % 64;
			len = sprintf(p_buf, "%crwxr-xr-x 1 ftp ftp %d %s %d %d %d:%.2d %s\r\n", temp_dir, fno.fsize, temp_mon[((fno.fdate >> 5) & 0x0f) - 1], (fno.fdate & 0x1f), (((fno.fdate >> 9) & 0x7f) + 1980),
						  h, m, fn);
			//printf("mon = %d, day = %d,  year = %d \r\n", ((fno.fdate >> 5) & 0x0f) - 1, (fno.fdate & 0x1f), (((fno.fdate >> 9) & 0x7f) + 1980));
			//printf("buf[%d]:%s", len, p_buf);
			p_buf += len;
		}
	}
	else
	{
		//printf("path(%s) not found:Error(%d)\r\n", path, res);
	}
	*p_buf = 0;
	*buf_len = strlen(buf1);
	//printf("last[%d]=%s[end]\r\n", *buf_len, buf1);
	return file_cnt;
}

int get_filesize(char *path, char *filename)
{
	FRESULT res;
	FILINFO fno;
	_DIR dir;
	int i, len, buf_ptr = 0;
	char *fn; /* This function is assuming no_Unicode cfg.*/
// #ifdef _USE_LFN
#if 0
	static char lfn[_MAX_LFN + 1];
	fno.lfname = lfn;
	fno.lfsize = sizeof(lfn);
#endif

	strcpy(buf, path); // res = f_opendir(&dir, path);
	res = fs_opendir();
#if defined(_FF_F_DEBUG_)
	printf("f_opendir res: %d\r\n", res);
#endif
	if (res == FR_OK)
	{
		for (;;)
		{
			res = fs_readdir(); // f_readdir(&dir, &fno);
			if (res != FR_OK || FS_DIRENTRY[DIR_Name] == 0)
				break;
			if (FS_DIRENTRY[DIR_Name] == '.')
				continue;
#if 0
#ifdef _USE_LFN
			fn = *fno.lfname ? fno.lfname : fno.fname;
#else
			fn = fno.fname;
#endif
#else
			get_file_name(fno.fname);
			fn = fno.fname;
#endif
			if (!strcmp(fn, filename))
			{
				static uint8_t attr;
				attr = FS_DIRENTRY[DIR_Attr];
				if (/*fno.fattrib*/ attr & 0x10)
				{
#if defined(_FF_F_DEBUG_)
					printf("\r\n%s/%s is a directory\r\n", path, filename);
#endif
					return 0;
				}
				memcpy(&fno.fsize, FS_DIRENTRY + DIR_FileSize, 4);
				return fno.fsize;
			}
		}
#if defined(_FF_F_DEBUG_)
		printf("\r\n%s/%s was not found\r\n", path, filename);
#endif
		// f_closedir(&dir);
	}
	return -1;
}

void set_fullpath(char *arg)
{
	size_t slen = strlen(arg);
	if (arg[slen - 2] == '\r')
		arg[slen - 2] = 0x00;
	if (arg[slen - 1] == '\n')
		arg[slen - 1] = 0x00;
	strcpy(buf, ftp.workingdir);
	if (strcmp(buf, "/") == 0)
		buf[0] = 0;
	if (strlen(buf) > 0)
		strcat(buf, "/");
	strcat(buf, arg);
}

uint8_t getSn_SR(int s)
{
	if (s == CTRL_SOCK)
		return  ftp.tcp_state ? ftp.tcp_state->sock_state : SOCK_CLOSED;
	if (s == DATA_SOCK)
		return  ftp.tcp_data_state ? ftp.tcp_data_state->sock_state : SOCK_CLOSED;
	return SOCK_CLOSED;
}

int close(int s)
{
	if (s == CTRL_SOCK)
	{
		tcp_ftp_server_close(ftp.tcp_state->client_pcb);
		return 0;
	}
	if (s == DATA_SOCK)
	{
		tcp_ftp_server_close(ftp.tcp_data_state->client_pcb);
		return 0;
	}
	return -1;
}

int getSn_RX_RSR(int s)
{
	if (s == CTRL_SOCK)
		return ftp_rcv_len;
	if (s == DATA_SOCK)
		return ftp_rcv_data_len;
	return 0;
}

int recv(int s, char *buf, int size)
{
	if (s == CTRL_SOCK)
	{
		memcpy(buf, ftp_rcv_buf, ftp_rcv_len);
		int s = ftp_rcv_len;
		ftp_rcv_len = 0;
		return s;
	}
	if (s == DATA_SOCK)
	{
		memcpy(buf, ftp_rcv_data_buf, ftp_rcv_data_len);
		int s = ftp_rcv_data_len;
		ftp_rcv_data_len = 0;
		return s;
	}
	return 0;
}

int disconnect(int s)
{
	if (s == CTRL_SOCK)
	{
		if (ftp.tcp_state->client_pcb)
		{
			err_t err = tcp_close(ftp.tcp_state->client_pcb);
			ftp.tcp_state->client_pcb = NULL;
			return err;
		}
		return 0;
	}
	if (s == DATA_SOCK)
	{
		if (ftp.tcp_data_state->client_pcb)
		{
			err_t err = tcp_close(ftp.tcp_data_state->client_pcb);
			ftp.tcp_data_state->client_pcb = NULL;
			return err;
		}
		return 0;
	}
	return -1;
}

#define SOCK_BUSY (-1)
#define SOCK_OK 0

int8_t socket(uint8_t sn, uint8_t protocol, uint16_t port, uint8_t flag)
{
	if (sn== CTRL_SOCK)
	{
		ftp.tcp_state = tcp_ftp_server_init();
		return CTRL_SOCK;
	}
	if (sn== DATA_SOCK)
	{
		ftp.tcp_data_state = tcp_ftp_server_init();
		return DATA_SOCK;
	}
	return -1;
}

int8_t listen(uint8_t s)
{
	if (s == CTRL_SOCK)
	{
       	if (!tcp_ftp_server_open(ftp.tcp_state, FTP_PORT))
	   		return -1;
		if (!tcp_ftp_server_listen(ftp.tcp_state))
			return -1;
		return 0;
	}
	if (s == DATA_SOCK)
	{
       	if (!tcp_ftp_server_open(ftp.tcp_data_state, local_port_actual))
	   		return -1;
		if (!tcp_ftp_server_listen(ftp.tcp_data_state))
			return -1;
		return 0;
	}
	return -1;

}

int8_t connect(uint8_t s, uint8_t* ip, uint16_t remote_port)
{
	if (s == DATA_SOCK)
	{
		memcpy(&ftp.tcp_data_state->remote_addr, ip, sizeof(ip_addr_t));
		ftp.tcp_data_state->remote_port = remote_port;
		if (!tcp_ftp_server_data_open(ftp.tcp_data_state))
	   		return -1;
		return 0;
	}
	return -1;
}

int send(int s, const uint8_t* pData, int size)
{
	if (s == CTRL_SOCK)
	{
		cyw43_arch_lwip_begin();
		tcp_write(ftp.tcp_state->client_pcb, pData, size, TCP_WRITE_FLAG_COPY);
		tcp_output(ftp.tcp_state->client_pcb);
		cyw43_arch_lwip_end();
		return size;
	}
	if (s == DATA_SOCK)
	{
		cyw43_arch_lwip_begin();
		tcp_write(ftp.tcp_data_state->client_pcb, pData, size, TCP_WRITE_FLAG_COPY);
		tcp_output(ftp.tcp_data_state->client_pcb);
		cyw43_arch_lwip_end();
		return size;
	}
	return 0;
}

void processFtpCmd(char *dbuf)
{
	int size = 0;
	int ret;
	switch (ftp.current_cmd)
	{
	case LIST_CMD:
	case MLSD_CMD:
#if defined(_FTP_DEBUG_)
		printf("previous size: %d\r\n", size);
#endif
		scan_files(ftp.workingdir, dbuf, (int *)&size);
#if defined(_FTP_DEBUG_)
		printf("returned size: %d\r\n", size);
		printf("%s\r\n", dbuf);
#endif
		size = strlen(dbuf);
		char *pData = dbuf;
		while (size > 0)
		{
			int sent = send(DATA_SOCK, pData, size);
			size -= sent;
			pData += sent;
		}
		ftp.current_cmd = NO_CMD;
		disconnect(DATA_SOCK);
		size = sprintf(dbuf, "226 Successfully transferred \"%s\"\r\n", ftp.workingdir);
		send(CTRL_SOCK, dbuf, size);
		break;

	case RETR_CMD:
#if defined(_FTP_DEBUG_)
		printf("filename to retrieve : %s %d\r\n", ftp.filename, strlen(ftp.filename));
#endif
		strcpy(buf, ftp.filename[0] == '/' ? ftp.filename + 1 : ftp.filename);
		ftp.fr = fs_open();
		// f_open(&(ftp.fil), (const char *)ftp.filename, FA_READ);
		// print_filedsc(&(ftp.fil));
		if (ftp.fr == FR_OK)
		{
			fs_getfilesize();
			DWORD remain_filesize = fs_tmp; // ftp.fil.fsize;
#if defined(_FTP_DEBUG_)
			printf("f_open return FR_OK\r\n");
#endif
			do
			{
#if defined(_FTP_DEBUG_)
				// printf("remained file size: %d\r\n", ftp.fil.fsize);
#endif
				memset(dbuf, 0, _MAX_SS);
                int send_byte = (remain_filesize > _MAX_SS) ? _MAX_SS : remain_filesize;

				ftp.fr = fs_read0(dbuf, send_byte); // f_read(&(ftp.fil), dbuf, send_byte , &blocklen);
				int blocklen = send_byte;
				if (ftp.fr != FR_OK)
					break;
#if defined(_FTP_DEBUG_)
				printf("#");
				// printf("----->fsize:%d recv:%d len:%d \r\n", remain_filesize, send_byte, blocklen);
				// printf("----->fn:%s data:%s \r\n", ftp.filename, dbuf);
#endif
				send(DATA_SOCK, dbuf, blocklen);
				remain_filesize -= blocklen;
			} while (remain_filesize != 0);
#if defined(_FTP_DEBUG_)
			printf("\r\nFile read finished\r\n");
#endif
			// ftp.fr = f_close(&(ftp.fil));
		}
		else
		{
#if defined(_FTP_DEBUG_)
			printf("File Open Error: %d\r\n", ftp.fr);
#endif
		}
		ftp.current_cmd = NO_CMD;
		disconnect(DATA_SOCK);
		size = sprintf(dbuf, "226 Successfully transferred \"%s\"\r\n", ftp.filename);
		send(CTRL_SOCK, dbuf, size);
		break;

	case STOR_CMD:
#if defined(_FTP_DEBUG_)
		printf("filename to store : %s %d\r\n", ftp.filename, strlen(ftp.filename));
#endif
		strcpy(buf, ftp.filename);
		fs_delete();
		strcpy(buf, ftp.filename);
		ftp.fr = fs_create(); // f_open(&(ftp.fil), (const char *)ftp.filename, FA_CREATE_ALWAYS | FA_WRITE);
		// print_filedsc(&(ftp.fil));
        BYTE err = ftp.fr;
		if (ftp.fr == FR_OK)
		{
#if defined(_FTP_DEBUG_)
			printf("f_open return FR_OK\r\n");
#endif
			while (1)
			{
				int remain_datasize;
				if ((remain_datasize = getSn_RX_RSR(DATA_SOCK)) > 0)
				{
					while (1)
					{
						memset(dbuf, 0, _MAX_SS);

						int recv_byte = (remain_datasize > _MAX_SS) ? _MAX_SS : remain_datasize;
						fs_wtotal = fs_file_wlen = recv_byte;
						ftp.fr = fs_write_start();						   // f_write(&(ftp.fil), dbuf, (UINT)ret, &blocklen);
						ret = recv(DATA_SOCK, fs_file_wbuf, fs_file_wlen); // dbuf, recv_byte);
						fs_file_wlen = ret;
						fs_wtotal = fs_file_wlen;
#if defined(_FTP_DEBUG_)
						// printf("----->fn:%s data:%s \r\n", ftp.filename, dbuf);
#endif

#if defined(_FTP_DEBUG_)
						// printf("----->dsize:%d recv:%d len:%d \r\n", remain_datasize, ret, blocklen);
#endif
						remain_datasize -= fs_wtotal;

						if (ftp.fr != FR_OK)
						{
#if defined(_FTP_DEBUG_)
							printf("f_write failed\r\n");
#endif
							break;
						}
						ftp.fr = fs_write_end();
						if (ftp.fr != FR_OK)
						{
#if defined(_FTP_DEBUG_)
							printf("f_write failed\r\n");
#endif
							break;
						}
						if (remain_datasize <= 0)
							break;
					}

					if (ftp.fr != FR_OK)
					{
#if defined(_FTP_DEBUG_)
						printf("f_write failed\r\n");
#endif
						break;
					}

#if defined(_FTP_DEBUG_)
					printf("#");
#endif
				}
				else
				{
					if (getSn_SR(DATA_SOCK) != SOCK_ESTABLISHED)
						break;
				}
			}
#if defined(_FTP_DEBUG_)
			printf("\r\nFile write finished\r\n");
#endif
			ftp.fr = fs_write_eof(); // f_close(&(ftp.fil));
		}
		else
		{
#if defined(_FTP_DEBUG_)
			printf("File Open Error: %d\r\n", ftp.fr);
#endif
			ftp.current_cmd = NO_CMD;
			disconnect(DATA_SOCK);
			size = sprintf(dbuf, "550 No such file or directory.\r\n");
			send(CTRL_SOCK, dbuf, size);
			break;
		}

		// fno.fdate = (WORD)(((current_year - 1980) << 9) | (current_month << 5) | current_day);
		// fno.ftime = (WORD)((current_hour << 11) | (current_min << 5) | (current_sec >> 1));
		// f_utime((const char *)ftp.filename, &fno);
		ftp.current_cmd = NO_CMD;
		disconnect(DATA_SOCK);
		size = sprintf(dbuf, "226 Successfully transferred \"%s\"\r\n", ftp.filename);
		send(CTRL_SOCK, dbuf, size);
		break;

	case NO_CMD:
	default:
		break;
	}
}

uint8_t ftpd_run(uint8_t *dbuf)
{
	int size = 0, i;
	long ret = 0;
	// memset(dbuf, 0, sizeof(_MAX_SS));

	switch (getSn_SR(CTRL_SOCK))
	{
	case SOCK_ESTABLISHED:
		if (!connect_state_control)
		{
#if defined(_FTP_DEBUG_)
			printf("%d:FTP Connected\r\n", CTRL_SOCK);
#endif
			// fsprintf(CTRL_SOCK, banner, HOSTNAME, VERSION);
			strcpy(ftp.workingdir, "/");
			sprintf((char *)dbuf, "220 %s FTP version %s ready.\r\n", HOSTNAME, VERSION);
		    cyw43_arch_lwip_begin();

			ret = send(CTRL_SOCK, (uint8_t *)dbuf, strlen((const char *)dbuf));
			cyw43_arch_lwip_end();
			if (ret < 0)
			{
#if defined(_FTP_DEBUG_)
				printf("%d:send() error:%ld\r\n", CTRL_SOCK, ret);
#endif
				close(CTRL_SOCK);
				return ret;
			}
			connect_state_control = 1;
		}

#if defined(_FTP_DEBUG_)
		printf("ftp socket %d\r\n", CTRL_SOCK);
#endif

		if ((size = getSn_RX_RSR(CTRL_SOCK)) > 0) // Don't need to check SOCKERR_BUSY because it doesn't not occur.
		{
#if defined(_FTP_DEBUG_)
			printf("size: %d\r\n", size);
#endif

			memset(dbuf, 0, FF_MAX_SS);

			if (size >= _MAX_SS)
				size = _MAX_SS - 1;

			ret = recv(CTRL_SOCK, dbuf, size);
			dbuf[ret] = '\0';
			if (ret != size)
			{
				if (ret == SOCK_BUSY)
					return 0;
				if (ret < 0)
				{
#if defined(_FTP_DEBUG_)
					printf("%d:recv() error:%ld\r\n", CTRL_SOCK, ret);
#endif
					close(CTRL_SOCK);
					return ret;
				}
			}
#if defined(_FTP_DEBUG_)
			printf("Rcvd Command: %s", dbuf);
#endif
			proc_ftpd((char *)dbuf);
		}
		break;

	case SOCK_CLOSE_WAIT:
#if defined(_FTP_DEBUG_)
		printf("%d:CloseWait\r\n", CTRL_SOCK);
#endif
		if ((ret = disconnect(CTRL_SOCK)) != SOCK_OK)
			return ret;
#if defined(_FTP_DEBUG_)
		printf("%d:Closed\r\n", CTRL_SOCK);
#endif
		break;

	case SOCK_CLOSED:
#if defined(_FTP_DEBUG_)
		printf("%d:FTPStart\r\n", CTRL_SOCK);
#endif
#define Sn_MR_TCP 0
		if ((ret = socket(CTRL_SOCK, Sn_MR_TCP, IPPORT_FTP, 0x0)) != CTRL_SOCK)
		{
#if defined(_FTP_DEBUG_)
			printf("%d:socket() error:%ld\r\n", CTRL_SOCK, ret);
#endif
			close(CTRL_SOCK);
			return ret;
		}
		break;

	case SOCK_INIT:
#if defined(_FTP_DEBUG_)
		printf("%d:Opened\r\n", CTRL_SOCK);
#endif
		// strcpy(ftp.workingdir, "/");
		if ((ret = listen(CTRL_SOCK)) != SOCK_OK)
		{
#if defined(_FTP_DEBUG_)
			printf("%d:Listen error\r\n", CTRL_SOCK);
#endif
			return ret;
		}
		connect_state_control = 0;

#if defined(_FTP_DEBUG_)
		printf("%d:Listen ok\r\n", CTRL_SOCK);
#endif
		break;

	default:
		break;
	}

#if 1
	switch (getSn_SR(DATA_SOCK))
	{
	case SOCK_ESTABLISHED:
		if (!connect_state_data)
		{
#if defined(_FTP_DEBUG_)
			printf("%d:FTP Data socket Connected\r\n", DATA_SOCK);
#endif
			connect_state_data = 1;
		}
        processFtpCmd(dbuf);
		break;

	case SOCK_CLOSE_WAIT:
#if defined(_FTP_DEBUG_)
		printf("%d:CloseWait\r\n", DATA_SOCK);
#endif
		if ((ret = disconnect(DATA_SOCK)) != SOCK_OK)
			return ret;
#if defined(_FTP_DEBUG_)
		printf("%d:Closed\r\n", DATA_SOCK);
#endif
		break;

	case SOCK_CLOSED:
		if (ftp.dsock_state == DATASOCK_READY)
		{
			if (ftp.dsock_mode == PASSIVE_MODE)
			{
#if defined(_FTP_DEBUG_)
				printf("%d:FTPDataStart, port : %d\r\n", DATA_SOCK, local_port);
#endif
				if ((ret = socket(DATA_SOCK, Sn_MR_TCP, local_port, 0x0)) != DATA_SOCK)
				{
#if defined(_FTP_DEBUG_)
					printf("%d:socket() error:%ld\r\n", DATA_SOCK, ret);
#endif
					close(DATA_SOCK);
					return ret;
				}
				local_port_actual = local_port;
				local_port++;
				if (local_port > 50000)
					local_port = 35000;
			}
			else
			{
#if defined(_FTP_DEBUG_)
				printf("%d:FTPDataStart, port : %d\r\n", DATA_SOCK, IPPORT_FTPD);
#endif
				if ((ret = socket(DATA_SOCK, Sn_MR_TCP, IPPORT_FTPD, 0x0)) != DATA_SOCK)
				{
#if defined(_FTP_DEBUG_)
					printf("%d:socket() error:%ld\r\n", DATA_SOCK, ret);
#endif
					close(DATA_SOCK);
					return ret;
				}
			}

			ftp.dsock_state = DATASOCK_START;
		}
		break;

	case SOCK_INIT:
#if defined(_FTP_DEBUG_)
		printf("%d:Opened\r\n", DATA_SOCK);
#endif
		if (ftp.dsock_mode == PASSIVE_MODE)
		{
			if ((ret = listen(DATA_SOCK)) != SOCK_OK)
			{
#if defined(_FTP_DEBUG_)
				printf("%d:Listen error\r\n", DATA_SOCK);
#endif
				return ret;
			}

#if defined(_FTP_DEBUG_)
			printf("%d:Listen ok\r\n", DATA_SOCK);
#endif
		}
		else
		{
			if ((ret = connect(DATA_SOCK, (remote_ip.cVal), remote_port)) != SOCK_OK)
			{
#if defined(_FTP_DEBUG_)
				printf("%d:Connect error\r\n", DATA_SOCK);
#endif
				return ret;
			}
		}
		connect_state_data = 0;
		break;

	default:
		break;
	}
#endif

	return 0;
}

inline int strlen_i(const char* arg)
{
	return (int)strlen(arg);
}


int getArg(char* arg)
{
	int slen = strlen_i(arg);
	if (slen >= 2)
		arg[slen - 2] = 0;
	return slen;
}


char proc_ftpd(char *ftp_buf)
{
	char **cmdp, *cp, *arg, *tmpstr;
	char sendbuf[200];
	int slen;
	long ret;

	/* Translate first word to lower case */
	for (cp = ftp_buf; *cp != ' ' && *cp != '\0'; cp++)
		*cp = tolower(*cp);

	/* Find command in table; if not present, return syntax error */
	for (cmdp = commands; *cmdp != NULL; cmdp++)
		if (strncmp(*cmdp, ftp_buf, strlen(*cmdp)) == 0)
			break;

	if (*cmdp == NULL)
	{
		// fsprintf(CTRL_SOCK, badcmd, ftp_buf);
		slen = sprintf(sendbuf, "500 Unknown command '%s'\r\n", ftp_buf);
		send(CTRL_SOCK, (uint8_t *)sendbuf, slen);
		return 0;
	}
	/* Allow only USER, PASS and QUIT before logging in */
	if (ftp.state == FTPS_NOT_LOGIN)
	{
		switch (cmdp - commands)
		{
		case USER_CMD:
		case PASS_CMD:
		case QUIT_CMD:
			break;
		default:
			// fsprintf(CTRL_SOCK, notlog);
			slen = sprintf(sendbuf, "530 Please log in with USER and PASS\r\n");
			send(CTRL_SOCK, (uint8_t *)sendbuf, slen);
			return 0;
		}
	}

	arg = &ftp_buf[strlen(*cmdp)];
	while (*arg == ' ')
		arg++;

	/* Execute specific command */
	enum ftp_cmd cmd = (enum ftp_cmd)(cmdp - commands);
	switch (cmd)
	{
	case USER_CMD:
#if defined(_FTP_DEBUG_)
		printf("USER_CMD : %s", arg);
#endif
		slen = getArg(arg);
		strcpy(ftp.username, arg);
		// fsprintf(CTRL_SOCK, givepass);
		slen = sprintf(sendbuf, "331 Enter PASS command\r\n");
		ret = send(CTRL_SOCK, (uint8_t *)sendbuf, slen);
		if (ret < 0)
		{
#if defined(_FTP_DEBUG_)
			printf("%d:send() error:%ld\r\n", CTRL_SOCK, ret);
#endif
			close(CTRL_SOCK);
			return ret;
		}
		break;

	case PASS_CMD:
#if defined(_FTP_DEBUG_)
		printf("PASS_CMD : %s", arg);
#endif
		slen = getArg(arg);
		ftplogin(arg);
		break;

	case TYPE_CMD:
		slen = getArg(arg);
		switch (arg[0])
		{
		case 'A':
		case 'a': /* Ascii */
			ftp.type = ASCII_TYPE;
			// fsprintf(CTRL_SOCK, typeok, arg);
			slen = sprintf(sendbuf, "200 Type set to %s\r\n", arg);
			send(CTRL_SOCK, (uint8_t *)sendbuf, slen);
			break;

		case 'B':
		case 'b': /* Binary */
		case 'I':
		case 'i': /* Image */
			ftp.type = IMAGE_TYPE;
			// fsprintf(CTRL_SOCK, typeok, arg);
			slen = sprintf(sendbuf, "200 Type set to %s\r\n", arg);
			send(CTRL_SOCK, (uint8_t *)sendbuf, slen);
			break;

		default: /* Invalid */
			// fsprintf(CTRL_SOCK, badtype, arg);
			slen = sprintf(sendbuf, "501 Unknown type \"%s\"\r\n", arg);
			send(CTRL_SOCK, (uint8_t *)sendbuf, slen);
			break;
		}
		break;

	case FEAT_CMD:
		slen = sprintf(sendbuf, "211-Features:\r\n MDTM\r\n REST STREAM\r\n SIZE\r\n MLST size*;type*;create*;modify*;\r\n MLSD\r\n UTF8\r\n CLNT\r\n MFMT\r\n211 END\r\n");
		send(CTRL_SOCK, (uint8_t *)sendbuf, slen);
		break;

	case QUIT_CMD:
#if defined(_FTP_DEBUG_)
		printf("QUIT_CMD\r\n");
#endif
		// fsprintf(CTRL_SOCK, bye);
		slen = sprintf(sendbuf, "221 Goodbye!\r\n");
		send(CTRL_SOCK, (uint8_t *)sendbuf, slen);
		disconnect(CTRL_SOCK);
		break;

	case RETR_CMD:
		slen = getArg(arg);
#if defined(_FTP_DEBUG_)
		printf("RETR_CMD\r\n");
#endif
		if (strcmp("/", ftp.workingdir) == 0)
			sprintf(ftp.filename, /*"/"*/ "%s", arg);
		else
			sprintf(ftp.filename, "%s/%s", ftp.workingdir, arg);
		slen = sprintf(sendbuf, "150 Opening data channel for file download from server of \"%s\"\r\n", ftp.filename);
		send(CTRL_SOCK, (uint8_t *)sendbuf, slen);
		ftp.current_cmd = RETR_CMD;
		break;

	case RNFR_CMD:
		set_fullpath(arg);
		strcpy(ftp.filename, buf);
		if (fs_openany() != 0)
		{
			slen = sprintf(sendbuf, "550 File does not exist\r\n");
		}
		else
		{
			slen = sprintf(sendbuf, "350 File exists, ready for destination name.\r\n");
			// strcpy(ftp.workingdir, arg);
		}
		send(CTRL_SOCK, (uint8_t *)sendbuf, slen);
		ftp.current_cmd = RNFR_CMD;
		break;

	case RNTO_CMD:
		strcpy(buf, ftp.filename);
		ftp.fr = fs_openany();
		if (ftp.fr == 0)
		{
			set_fullpath(arg);
			ftp.fr = fs_move0();
		}
		if (ftp.fr != 0)
		{
			slen = sprintf(sendbuf, "550 Unknown error.\r\n");
		}
		else
		{
			slen = sprintf(sendbuf, "250 File or directory renamed successfully.\r\n");
		}
		send(CTRL_SOCK, (uint8_t *)sendbuf, slen);
		ftp.current_cmd = RNTO_CMD;
		break;

	case APPE_CMD:
	case STOR_CMD:
		slen = getArg(arg);
#if defined(_FTP_DEBUG_)
		printf("STOR_CMD\r\n");
#endif
		if (strcmp("/", ftp.workingdir) == 0)
			sprintf(ftp.filename, /*"/"*/ "%s", arg);
		else
			sprintf(ftp.filename, "%s/%s", ftp.workingdir, arg);
		slen = sprintf(sendbuf, "150 Opening data channel for file upload to server of \"%s\"\r\n", ftp.filename);
		send(CTRL_SOCK, (uint8_t *)sendbuf, slen);
		ftp.current_cmd = STOR_CMD;
		if ((ret = connect(DATA_SOCK, remote_ip.cVal, remote_port)) != SOCK_OK)
		{
#if defined(_FTP_DEBUG_)
			printf("%d:Connect error\r\n", DATA_SOCK);
#endif
			return ret;
		}
		connect_state_data = 0;
		break;

	case PORT_CMD:
#if defined(_FTP_DEBUG_)
		printf("PORT_CMD\r\n");
#endif
		if (pport(arg) == -1)
		{
			// fsprintf(CTRL_SOCK, badport);
			slen = sprintf(sendbuf, "501 Bad port syntax\r\n");
			send(CTRL_SOCK, (uint8_t *)sendbuf, slen);
		}
		else
		{
			// fsprintf(CTRL_SOCK, portok);
			ftp.dsock_mode = ACTIVE_MODE;
			ftp.dsock_state = DATASOCK_READY;
			slen = sprintf(sendbuf, "200 PORT command successful.\r\n");
			send(CTRL_SOCK, (uint8_t *)sendbuf, slen);
		}
		break;

	case MLSD_CMD:
#if defined(_FTP_DEBUG_)
		printf("MLSD_CMD\r\n");
#endif
		slen = sprintf(sendbuf, "150 Opening data channel for directory listing of \"%s\"\r\n", ftp.workingdir);
		send(CTRL_SOCK, (uint8_t *)sendbuf, slen);
		ftp.current_cmd = MLSD_CMD;
		break;

	case LIST_CMD:
#if defined(_FTP_DEBUG_)
		printf("LIST_CMD\r\n");
#endif
		slen = sprintf(sendbuf, "150 Opening data channel for directory listing of \"%s\"\r\n", ftp.workingdir);
		send(CTRL_SOCK, (uint8_t *)sendbuf, slen);
		ftp.current_cmd = LIST_CMD;
		break;

	case NLST_CMD:
#if defined(_FTP_DEBUG_)
		printf("NLST_CMD\r\n");
#endif
		break;

	case SYST_CMD:
		slen = sprintf(sendbuf, "215 UNIX emulated by WIZnet\r\n");
		send(CTRL_SOCK, (uint8_t *)sendbuf, slen);
		break;

	case PWD_CMD:
	case XPWD_CMD:
		slen = sprintf(sendbuf, "257 \"%s\" is current directory.\r\n", ftp.workingdir);
		send(CTRL_SOCK, (uint8_t *)sendbuf, slen);
		break;

	case PASV_CMD:
		slen = sprintf(sendbuf, "227 Entering Passive Mode (%d,%d,%d,%d,%d,%d)\r\n", local_ip.cVal[0], local_ip.cVal[1], local_ip.cVal[2], local_ip.cVal[3], local_port >> 8, local_port & 0x00ff);
		send(CTRL_SOCK, (uint8_t *)sendbuf, slen);
		disconnect(DATA_SOCK);
		ftp.dsock_mode = PASSIVE_MODE;
		ftp.dsock_state = DATASOCK_READY;
#if defined(_FTP_DEBUG_)
		printf("PASV port: %d\r\n", local_port);
#endif
		break;

	case SIZE_CMD:
		slen = getArg(arg);
		if (slen > 3)
		{
			tmpstr = strrchr(arg, '/');
			*tmpstr = 0;

			if (strlen(ftp.workingdir) == 1)
				sprintf(ftp.filename, "/%s", arg);
			else
				sprintf(ftp.filename, "%s/%s", ftp.workingdir, arg);
			strcpy(buf, ftp.filename[0] == '/' ? ftp.filename + 1 : ftp.filename);
			fs_open();
			fs_getfilesize(); // get_filesize(arg, tmpstr + 1);
			slen = fs_tmp;
			fs_close();
			if (slen > 0)
				slen = sprintf(sendbuf, "213 %d\r\n", slen);
			else
				slen = sprintf(sendbuf, "550 File not Found\r\n");
		}
		else
		{
			slen = sprintf(sendbuf, "550 File not Found\r\n");
		}
		send(CTRL_SOCK, (uint8_t *)sendbuf, slen);
		break;

	case CWD_CMD:
		slen = getArg(arg);
		if (slen > 3)
		{
			// arg[slen - 3] = 0x00;
			tmpstr = strrchr(arg, '/');
			if (strcmp(arg, "..") == 0)
			{
				*arg = 0;
				tmpstr = strrchr(ftp.workingdir, '/');
				if (tmpstr)
					*tmpstr = 0;
				else
					strcpy(ftp.workingdir, "/");
				slen = 0;
			}
			else
			{
				if (tmpstr == NULL)
					slen = get_filesize(ftp.workingdir, arg);
				else
					*tmpstr = 0;
				if (tmpstr != NULL)
					slen = get_filesize(arg, tmpstr + 1);
				if (slen == -1)
					slen = 0;
				if (tmpstr)
					*tmpstr = '/';
			}
			if (slen == 0)
			{
				slen = sprintf(sendbuf, "213 %d\r\n", slen);
				// strcpy(ftp.workingdir, arg);
				if (strcmp(ftp.workingdir, "/") && strlen(ftp.workingdir) != 0)
				{
					if (strlen(arg))
						strcat(ftp.workingdir, "/");
				}
				else
					ftp.workingdir[0] = 0;
				strcat(ftp.workingdir, arg);
				slen = sprintf(sendbuf, "250 CWD successful. \"%s\" is current directory.\r\n", ftp.workingdir);
			}
			else
			{
				slen = sprintf(sendbuf, "550 CWD failed. \"%s\"\r\n", arg);
			}
		}
		else
		{
			strcpy(ftp.workingdir, arg);
			slen = sprintf(sendbuf, "250 CWD successful. \"%s\" is current directory.\r\n", ftp.workingdir);
		}
		send(CTRL_SOCK, (uint8_t *)sendbuf, slen);
		break;

	case MKD_CMD:
	case XMKD_CMD:
		slen = getArg(arg);
		// strcpy(buf, arg);
		set_fullpath(arg);
		if (fs_createdir() != 0) // f_mkdir(arg) != 0)
		{
			slen = sprintf(sendbuf, "550 Can't create directory. \"%s\"\r\n", arg);
		}
		else
		{
			slen = sprintf(sendbuf, "257 MKD command successful. \"%s\"\r\n", arg);
			// strcpy(ftp.workingdir, arg);
		}
		send(CTRL_SOCK, (uint8_t *)sendbuf, slen);
		break;

	case DELE_CMD:
		slen = getArg(arg);
		set_fullpath(arg);
		if (fs_delete() != 0) // f_unlink(arg) != 0)
		{
			slen = sprintf(sendbuf, "550 Could not delete. \"%s\"\r\n", arg);
		}
		else
		{
			slen = sprintf(sendbuf, "250 Deleted. \"%s\"\r\n", arg);
		}
		send(CTRL_SOCK, (uint8_t *)sendbuf, slen);
		break;

	case XCWD_CMD:
	case ACCT_CMD:
	case XRMD_CMD:
	case RMD_CMD:
	case STRU_CMD:
	case MODE_CMD:
	case XMD5_CMD:
		// fsprintf(CTRL_SOCK, unimp);
		slen = sprintf(sendbuf, "502 Command does not implemented yet.\r\n");
		send(CTRL_SOCK, (uint8_t *)sendbuf, slen);
		break;

	default: /* Invalid */
		// fsprintf(CTRL_SOCK, badcmd, arg);
		slen = sprintf(sendbuf, "500 Unknown command \'%s\'\r\n", arg);
		send(CTRL_SOCK, (uint8_t *)sendbuf, slen);
		break;
	}

	return 1;
}

char ftplogin(char *pass)
{
	char sendbuf[100];
	int slen = 0;

	// memset(sendbuf, 0, DATA_BUF_SIZE);

#if defined(_FTP_DEBUG_)
	printf("%s logged in\r\n", ftp.username);
#endif
	// fsprintf(CTRL_SOCK, logged);
	slen = sprintf(sendbuf, "230 Logged on\r\n");
	send(CTRL_SOCK, (uint8_t *)sendbuf, slen);
	ftp.state = FTPS_LOGIN;

	return 1;
}

int pport(char *arg)
{
	int i;
	char *tok = 0;

	for (i = 0; i < 4; i++)
	{
		if (i == 0)
			tok = strtok(arg, ",\r\n");
		else
			tok = strtok(NULL, ",");
		remote_ip.cVal[i] = (uint8_t)atoi(tok);
		if (!tok)
		{
#if defined(_FTP_DEBUG_)
			printf("bad pport : %s\r\n", arg);
#endif
			return -1;
		}
	}
	remote_port = 0;
	for (i = 0; i < 2; i++)
	{
		tok = strtok(NULL, ",\r\n");
		remote_port <<= 8;
		remote_port += atoi(tok);
		if (!tok)
		{
#if defined(_FTP_DEBUG_)
			printf("bad pport : %s\r\n", arg);
#endif
			return -1;
		}
	}
#if defined(_FTP_DEBUG_)
	printf("ip : %d.%d.%d.%d, port : %d\r\n", remote_ip.cVal[0], remote_ip.cVal[1], remote_ip.cVal[2], remote_ip.cVal[3], remote_port);
#endif

	return 0;
}

void print_filedsc(FIL *fil)
{
#if defined(_FTP_DEBUG_)
	printf("File System pointer : %08X\r\n", fil->fs);
	printf("File System mount ID : %d\r\n", fil->id);
	printf("File status flag : %08X\r\n", fil->flag);
	printf("File System pads : %08X\r\n", fil->err);
	printf("File read write pointer : %08X\r\n", fil->fptr);
	printf("File size : %08X\r\n", fil->fsize);
	printf("File start cluster : %08X\r\n", fil->sclust);
	printf("current cluster : %08X\r\n", fil->clust);
	printf("current data sector : %08X\r\n", fil->dsect);
	printf("dir entry sector : %08X\r\n", fil->dir_sect);
	printf("dir entry pointer : %08X\r\n", fil->dir_ptr);
#endif
}
