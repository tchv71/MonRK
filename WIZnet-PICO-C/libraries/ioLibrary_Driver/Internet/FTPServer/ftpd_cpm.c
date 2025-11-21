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
#include "socket.h"
#include "ftpd_cpm.h"
#include <sys/fcntl.h>

/* If you need this header, use it. */
// #include "stdio_private.h"

/* Command table */
extern char *commands[];


static un_l2cval remote_ip;
static uint16_t remote_port;
static un_l2cval local_ip;
static uint16_t local_port;
static uint16_t local_port_old;
static uint8_t connect_state_control = 0;
static uint8_t connect_state_data = 0;

static struct ftpd ftp;

static int current_year = 2014;
static int current_month = 12;
static int current_day = 31;
static int current_hour = 10;
static int current_min = 10;
static int current_sec = 30;


typedef int FRESULT;
#define FR_OK 0
extern void strip_spaces(char *str, char *right);
void get_file_name_cpm(char* fname, struct cpmDirent* dirent)
{
	strcpy(fname, dirent->name + 2);
}

static struct cpmSuperBlock drive;
static struct cpmInode root;
static bool bMounted = false;
static struct cpmFile dir;
static struct cpmStatFS bufStat;

char image_name[128];

extern int get_file_name(char *);

FRESULT unmount_drive();

FRESULT mount_drive()
{
	unmount_drive();
	FRESULT res;
	if (bMounted)
		return FR_OK;
	const char *err;
	const char *image = "CPM/A.KDI";
	const char *format = "17153";
	const char *devopts = NULL;

	if (strcmp(ftp.workingdir, "/") == 0)
		return FR_OK;
	strcpy(image_name, "CPM/");
	strcat(image_name, ftp.username);
	/* open image */ /*{{{*/
	if ((err = Device_open(&drive.dev, image_name, O_RDWR, devopts)))
		return -1;

	// 17153 format
	struct cpmSuperBlock* d = &drive;
	if (strncmp(ftp.filename, "C", 1) != 0)
	{
		d->secLength = 1024;
		d->tracks = 160;
		d->sectrk = 5;
		d->blksiz = 2048;
		d->maxdir = 128;
		d->skew = 0;
		d->skewtab = NULL;
		d->type = 0;
		d->boottrk = 4;
		d->extents = 1;
	}
	else
	{
//   seclen 512
//   tracks 1000
//   sectrk 16
//   blocksize 4096
//   maxdir 1024
//   skew 0
//   boottrk 2
		d->secLength = 512;
		d->tracks = 1000;
		d->sectrk = 16;
		d->blksiz = 4096;
		d->maxdir = 1024;
		d->skew = 0;
		d->skewtab = NULL;
		d->type = 0;
		d->boottrk = 2;
		d->extents = 1;
	}
	d->size = (d->secLength * d->sectrk * (d->tracks - d->boottrk)) / d->blksiz;

	if ((res = cpmReadSuper(&drive, &root, NULL/*format*/)) != FR_OK)
		return res;	
    bMounted = true;
	return FR_OK;
}

FRESULT unmount_drive()
{
	if (bMounted)
	{
		const char* res = Device_close(&drive.dev);
		cpmUmount(&drive);
		bMounted = false;
	}
	return FR_OK;
}

FRESULT scan_files_cpm(char* path, char* buf1, int* buf_len, const char *pattern)
{
	int res = 0;
	char isDir = 0;
	if (strcmp(ftp.workingdir, "/") == 0)
	{
		isDir = 1;
		pattern = "*.KDI";
		strcpy(buf, "CPM");
		unmount_drive();
		res = fs_opendir();
		if (res)
			return res;
	}
	else
	{
		mount_drive();
		if ((res = cpmOpendir(&root, &dir)) != FR_OK)
			return res;
		cpmStatFS(&root, &bufStat);
	}

	unsigned int file_cnt = 0;
	FILINFO fno;
	char* p_buf = buf1;
	char temp_dir = 0;
	WORD temp_f_date = 0;
	WORD temp_f_time = 0;
  	struct cpmDirent dirent;
	memset(&dirent, 0, sizeof dirent);
	while (isDir || cpmReaddir(&dir, &dirent))
	{
		struct cpmStat statbuf;
		struct cpmInode file;
		char temp_mon[12][4] = { "JAN", "FEB", "MAR", "APR", "MAY", "JUN", "JUL", "AUG", "SEP", "OCT", "NOV", "DEC" };

		if (isDir)
		{
			res = fs_readdir(); // f_readdir(&dir, &fno);
			if (res != FR_OK || FS_DIRENTRY[0] == 0)
				break;
			if (FS_DIRENTRY[0] == '.')
				continue;
			char* pExt = FS_DIRENTRY + DIR_Name + 8;
			strupr(pExt);
			if (strncmp(pExt, "KDI", 3) != 0)
			{
				continue;
			}
			get_file_name(fno.fname);
		}
		else
		{
			if (strcmp(dirent.name, ".") == 0 || strcmp(dirent.name, "..") == 0)
				continue;
			if (pattern && !match(dirent.name, pattern))
				continue;
			get_file_name_cpm(fno.fname, &dirent);
		}

		++file_cnt;
		if (ftp.current_cmd != NLST_CMD)
		{
			cpmNamei(&root, dirent.name, &file);
			cpmStat(&file, &statbuf);
			fno.fsize = statbuf.size;// +bufStat.f_bsize - 1) / bufStat.f_bsize * (bufStat.f_bsize / 1024));
			temp_dir = '-';
			if (isDir)
			    temp_dir = 'd';
			fno.fdate = temp_f_date;
			fno.ftime = temp_f_time;
			uint8_t h = fno.ftime >> 11;
			uint8_t m = (fno.ftime >> 5) % 64;
			int mon = ((fno.fdate >> 5) & 0x0f) - 1;
			if (mon < 0)
				mon = 0;
			WORD date = fno.fdate & 0x1f;
			if (date == 0)
				date = 1;
			int len = sprintf(p_buf, "%crwxr-xr-x 1 ftp ftp %d %s %d %d %d:%.2d %s\r\n", temp_dir, fno.fsize, temp_mon[mon], date, (((fno.fdate >> 9) & 0x7f) + 1980),
				h, m, fno.fname);
			p_buf += len;
		}
		else
		{
			int len = sprintf(p_buf, "%s\r\n", fno.fname);
			p_buf += len;
		}
	}
	*p_buf = 0;
	*buf_len = (int)strlen(buf1);
	return file_cnt;
}

int get_filesize_cpm(char* path, char* filename)
{
    mount_drive();
	int res;
	if ((res = cpmOpendir(&root, &dir)) != FR_OK)
		return res;
  	struct cpmDirent dirent;
	FILINFO fno;
	while (cpmReaddir(&dir, &dirent))
	{
		if (strcmp(dirent.name, ".") == 0 || strcmp(dirent.name, "..") == 0)
			continue;
		struct cpmStat statbuf;
		struct cpmInode file;
		get_file_name_cpm(fno.fname, &dirent);
		if (strcmp(fno.fname, filename) == 0)
		{
			cpmNamei(&root, dirent.name, &file);
			cpmStat(&file, &statbuf);
			fno.fsize = (long)statbuf.size;
			return fno.fsize;
		}
	}
	return -1;
}

void set_fullpath_cpm(char* arg)
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

#define LOCAL_PORT 45000

void ftpd_cpm_init(uint8_t* src_ip)
{
	ftp.state = FTPS_NOT_LOGIN;
	ftp.current_cmd = NO_CMD;
	ftp.dsock_mode = ACTIVE_MODE;

	local_ip.cVal[0] = src_ip[0];
	local_ip.cVal[1] = src_ip[1];
	local_ip.cVal[2] = src_ip[2];
	local_ip.cVal[3] = src_ip[3];
	local_port_old = local_port = LOCAL_PORT;

	strcpy(ftp.workingdir, "/");
}


long ftpd_cpm_run(uint8_t* dbuf)
{
	int size = 0;
	long ret = 0;
	int32_t remain_datasize;

	// memset(dbuf, 0, sizeof(_MAX_SS));

	switch (getSn_SR(CTRL_SOCK_CPM))
	{
	case SOCK_ESTABLISHED:
		if (!connect_state_control)
		{
#if defined(_FTP_DEBUG_)
			printf("%d:FTP Connected\r\n", (int)CTRL_SOCK_CPM);
#endif
			// fsprintf(CTRL_SOCK_CPM, banner, HOSTNAME, VERSION);
			strcpy(ftp.workingdir, "/");
			sprintf((char*)dbuf, "220 %s FTP version %s ready.\r\n", HOSTNAME, VERSION);
			ret = send(CTRL_SOCK_CPM, (uint8_t*)dbuf, (int)strlen((const char*)dbuf));
			if (ret < 0)
			{
#if defined(_FTP_DEBUG_)
				printf("%d:send() error:%ld\r\n", (int)CTRL_SOCK_CPM, ret);
#endif
				close(CTRL_SOCK_CPM);
				return ret;
			}
			connect_state_control = 1;
		}

#if defined(_FTP_DEBUG_)
		//printf("ftp socket %d\r\n", (int)CTRL_SOCK_CPM);
#endif

		if ((size = getSn_RX_RSR(CTRL_SOCK_CPM)) > 0) // Don't need to check SOCKERR_BUSY because it doesn't not occur.
		{
#if defined(_FTP_DEBUG_)
			printf("size: %d\r\n", size);
#endif

			memset(dbuf, 0, FF_MAX_SS);

			if (size > _MAX_SS)
				size = _MAX_SS - 1;

			ret = recv(CTRL_SOCK_CPM, dbuf, size);
			dbuf[ret] = '\0';
			if (ret != size)
			{
				if (ret == 0/*SOCK_BUSY*/)
					return 0;
				if (ret < 0)
				{
#if defined(_FTP_DEBUG_)
					printf("%d:recv() error:%ld\r\n", (int)CTRL_SOCK_CPM, ret);
#endif
					close(CTRL_SOCK_CPM);
					return ret;
				}
			}
#if defined(_FTP_DEBUG_)
			printf("Rcvd Command: %s", dbuf);
#endif
			proc_ftpd_cpm((char*)dbuf);
		}
		break;

	case SOCK_CLOSE_WAIT:
#if defined(_FTP_DEBUG_)
		printf("%d:CloseWait\r\n", (int)CTRL_SOCK_CPM);
#endif
		if ((ret = close(CTRL_SOCK_CPM)) != SOCK_OK)
			return ret;
#if defined(_FTP_DEBUG_)
		printf("%d:Closed\r\n", (int)CTRL_SOCK_CPM);
#endif
		break;

	case SOCK_CLOSED:
#if defined(_FTP_DEBUG_)
		printf("%d:FTPStart\r\n", (int)CTRL_SOCK_CPM);
#endif
		if ((ret = socket(CTRL_SOCK_CPM, Sn_MR_TCP, IPPORT_FTP+1000, 0x0)) != CTRL_SOCK_CPM)
		{
#if defined(_FTP_DEBUG_)
			printf("%d:socket() error:%ld\r\n", (int)CTRL_SOCK_CPM, ret);
#endif
			close(CTRL_SOCK_CPM);
			return ret;
		}
		break;

	case SOCK_INIT:
	{
#if defined(_FTP_DEBUG_)
		//printf("%d:Opened\r\n", (int)CTRL_SOCK_CPM);
#endif
		// strcpy(ftp.workingdir, "/");
		if ((ret = listen(CTRL_SOCK_CPM)) != SOCK_OK)
		{
#if defined(_FTP_DEBUG_)
			printf("%d:Listen error\r\n", CTRL_SOCK_CPM);
#endif
			return ret;
		}
		connect_state_control = 0;

#if defined(_FTP_DEBUG_)
		printf("%d:Listen ok\r\n", (int)CTRL_SOCK_CPM);
#endif
	}
	break;

	default:
		break;
	}

#if 1
	switch (getSn_SR(DATA_SOCK_CPM))
	{
	case SOCK_ESTABLISHED:
		if (!connect_state_data)
		{
#if defined(_FTP_DEBUG_)
			printf("%d:FTP Data socket Connected\r\n", (int)DATA_SOCK_CPM);
#endif
			connect_state_data = 1;
		}

		switch (ftp.current_cmd)
		{
		case LIST_CMD:
		case MLSD_CMD:
		case NLST_CMD:
#if defined(_FTP_DEBUG_)
			printf("previous size: %d\r\n", size);
#endif
			mount_drive();
			scan_files_cpm(ftp.workingdir, dbuf, (int*)&size, strlen(ftp.filename)>0 ? ftp.filename : NULL);
#if defined(_FTP_DEBUG_)
			printf("returned size: %d\r\n", size);
			printf("%s\r\n", dbuf);
#endif
			size = (int)strlen(dbuf);
			char* pData = dbuf;
			while (size > 0)
			{
				int sent = send(DATA_SOCK_CPM, pData, size);
				if (sent < 0)
				{
					close(DATA_SOCK_CPM);
					size = sprintf(dbuf, "550 Failed to transfer\"%s\"\r\n", ftp.workingdir);
					send(CTRL_SOCK_CPM, dbuf, size);
					break;
				}
				size -= sent;
				pData += sent;
			}
			ftp.current_cmd = NO_CMD;
			close(DATA_SOCK_CPM);
			size = sprintf(dbuf, "226 Successfully transferred \"%s\"\r\n", ftp.workingdir);
			send(CTRL_SOCK_CPM, dbuf, size);
			break;

		case RETR_CMD:
		{
#if defined(_FTP_DEBUG_)
			printf("filename to retrieve : %s %d\r\n", ftp.filename, (int)strlen(ftp.filename));
#endif
			mount_drive();
			strcpy(buf, ftp.filename[0] == '/' ? ftp.filename + 1 : ftp.filename);
			//ftp.fr = fs_open();
			struct cpmInode ino;
			char cpmname[2 + 8 + 1 + 3 + 1]; /* 00foobarxy.zzy\0 */
			toCpmName(cpmname, buf);

			if (cpmNamei(&root, cpmname, &ino) == FR_OK)
			{
				struct cpmFile file;

				if (cpmOpen(&ino, &file, O_RDONLY) == FR_OK)
				{
					int crpending = 0;
					int ohno = 0;
					int res;
					char fbuf[4096];

					while ((res = cpmRead(&file, fbuf, sizeof(fbuf))) > 0)
					{
						int remain = res;
						char *ptr = fbuf;
						while (remain > 0)
						{
							int sent = send(DATA_SOCK_CPM, ptr, remain);
							if (sent < 0)
							{
								res = sent;
								return -1;
							}
							remain -= sent;
							ptr += sent;
						}
					}
				}
				cpmClose(&file);
			}
			ftp.current_cmd = NO_CMD;
			close(DATA_SOCK_CPM);
			size = sprintf(dbuf, "226 Successfully transferred \"%s\"\r\n", ftp.filename);
			send(CTRL_SOCK_CPM, dbuf, size);
		}
		break;

		case STOR_CMD:
		{
#if defined(_FTP_DEBUG_)
			printf("filename to store : %s %d\r\n", ftp.filename, (int)strlen(ftp.filename));
#endif
			unmount_drive();
			mount_drive();
			char cpmname[2 + 8 + 1 + 3 + 1]; /* 00foobarxy.zzy\0 */
			toCpmName(cpmname, ftp.filename);
			cpmUnlink(&root, cpmname);
			struct cpmInode ino;
			ftp.fr = cpmCreat(&root, cpmname, &ino, 0666); // f_open(&(ftp.fil), (const char *)ftp.filename, FA_CREATE_ALWAYS | FA_WRITE);
			// print_filedsc(&(ftp.fil));
			if (ftp.fr == FR_OK)
			{
#if defined(_FTP_DEBUG_)
				printf("f_open return FR_OK\r\n");
#endif
				struct cpmFile file;
				cpmOpen(&ino, &file, O_WRONLY);
				while (1)
				{
					int i = 0;
					sleep_ms(100);
					do
					{
						remain_datasize = getSn_RX_RSR(DATA_SOCK_CPM);
					} while (remain_datasize == 0 && ++i < 10);
					if (remain_datasize > 0)
					{
						while (1)
						{
							//memset(dbuf, 0, _MAX_SS);

							int received = remain_datasize;
							if (remain_datasize > ETHERNET_BUF_MAX_SIZE)
								received = ETHERNET_BUF_MAX_SIZE;
							//else
							received = recv(DATA_SOCK_CPM, dbuf, received);
							ftp.fr = cpmWrite(&file, dbuf, received) != received ? 1 : 0;
#if defined(_FTP_DEBUG_)
							printf("----->fn:%s data:%s \r\n", ftp.filename, dbuf);
#endif

#if defined(_FTP_DEBUG_)
							printf("----->dsize:%d recv:%d len:%d \r\n", remain_datasize, ret, j);
#endif
							remain_datasize -= received;

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
						if (getSn_SR(DATA_SOCK_CPM) != SOCK_ESTABLISHED)
							break;
					}
				}
#if defined(_FTP_DEBUG_)
				printf("\r\nFile write finished\r\n");
#endif
				ftp.fr = cpmClose(&file); // f_close(&(ftp.fil));
				int nRes = cpmSync(&drive);
				unmount_drive();
			}
			else
			{
#if defined(_FTP_DEBUG_)
				printf("File Open Error: %d\r\n", ftp.fr);
#endif
				ftp.current_cmd = NO_CMD;
				close(DATA_SOCK_CPM);
				size = sprintf(dbuf, "550 No such file or directory.\r\n");
				send(CTRL_SOCK_CPM, dbuf, size);
				break;
			}

			// fno.fdate = (WORD)(((current_year - 1980) << 9) | (current_month << 5) | current_day);
			// fno.ftime = (WORD)((current_hour << 11) | (current_min << 5) | (current_sec >> 1));
			// f_utime((const char *)ftp.filename, &fno);
			ftp.current_cmd = NO_CMD;
			close(DATA_SOCK_CPM);
			size = sprintf(dbuf, "226 Successfully transferred \"%s\"\r\n", ftp.filename);
			send(CTRL_SOCK_CPM, dbuf, size);
		}
		break;

		case NO_CMD:
		default:
			break;
		}
		break;

	case SOCK_CLOSE_WAIT:
#if defined(_FTP_DEBUG_)
		printf("%d:CloseWait\r\n", (int)DATA_SOCK_CPM);
#endif
		if ((ret = close(DATA_SOCK_CPM)) != SOCK_OK)
			return ret;
#if defined(_FTP_DEBUG_)
		printf("%d:Closed\r\n", (int)DATA_SOCK_CPM);
#endif
		break;

	case SOCK_CLOSED:
		if (ftp.dsock_state == DATASOCK_READY)
		{
			if (ftp.dsock_mode == PASSIVE_MODE)
			{
#if defined(_FTP_DEBUG_)
				printf("%d:FTPDataStart, port : %d\r\n", (int)DATA_SOCK_CPM, local_port);
#endif
				if ((ret = socket(DATA_SOCK_CPM, Sn_MR_TCP, local_port, 0x0)) != DATA_SOCK_CPM)
				{
#if defined(_FTP_DEBUG_)
					printf("%d:socket() error:%ld\r\n", DATA_SOCK_CPM, ret);
#endif
					close(DATA_SOCK_CPM);
					return ret;
				}

				local_port_old = local_port;
				local_port++;
				if (local_port > 50000)
					local_port = LOCAL_PORT;
			}
			else
			{
#if defined(_FTP_DEBUG_)
				printf("%d:FTPDataStart, port : %d\r\n", (int)DATA_SOCK_CPM, IPPORT_FTPD);
#endif
				if ((ret = socket(DATA_SOCK_CPM, Sn_MR_TCP, IPPORT_FTPD, 0x0)) != DATA_SOCK_CPM)
				{
#if defined(_FTP_DEBUG_)
					printf("%d:socket() error:%ld\r\n", (int)DATA_SOCK_CPM, ret);
#endif
					close(DATA_SOCK_CPM);
					return ret;
				}
			}

			ftp.dsock_state = DATASOCK_START;
		}
		break;

	case SOCK_INIT:
#if defined(_FTP_DEBUG_)
		//printf("%d:Opened\r\n", (int)DATA_SOCK_CPM);
#endif
		if (ftp.dsock_mode == PASSIVE_MODE)
		{
			if ((ret = listen(DATA_SOCK_CPM)) != SOCK_OK)
			{
#if defined(_FTP_DEBUG_)
				printf("%d:Listen error\r\n", DATA_SOCK_CPM);
#endif
				return ret;
			}

#if defined(_FTP_DEBUG_)
			printf("%d:Listen ok\r\n", DATA_SOCK_CPM);
#endif
		}
		else
		{
			if (remote_ip.lVal == 0)
			{
				return 0;
			}
			if ((ret = connect(DATA_SOCK_CPM, remote_ip.cVal, remote_port)) != SOCK_OK)
			{
#if defined(_FTP_DEBUG_)
				printf("%d:Connect error\r\n", DATA_SOCK_CPM);
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

void toCpmName(char cpmname[15], const char* filename)
{
	strcpy(cpmname, "00");
	char fname[13];
	char *fname2 = strrchr(filename, '/') + 1;
	strcpy(fname, fname2);
	char* pDot = strchr(fname, '.');
	int len = pDot ? pDot - fname : strlen(fname);
	if (len>8)
		len = 8;
	strncpy(cpmname + 2, fname, len);
	cpmname[2 + len] = 0;
	if (pDot)
	{
		strcat(cpmname, ".");
		strncpy(cpmname + 2 + 1 + len, pDot + 1, 3);
		cpmname[2 + 1 + len + 3] = 0;
	}
}

extern int getArg(char* arg);

long proc_ftpd_cpm(char* ftp_buf)
{
	char** cmdp, * cp, * arg/*, * tmpstr*/;
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
		// fsprintf(CTRL_SOCK_CPM, badcmd, ftp_buf);
		slen = sprintf(sendbuf, "500 Unknown command '%s'\r\n", ftp_buf);
		send(CTRL_SOCK_CPM, (uint8_t *)sendbuf, slen);
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
			// fsprintf(CTRL_SOCK_CPM, notlog);
			slen = sprintf(sendbuf, "530 Please log in with USER and PASS\r\n");
			send(CTRL_SOCK_CPM, (uint8_t *)sendbuf, slen);
			return 0;
		}
	}

	arg = &ftp_buf[strlen(*cmdp)];
	while (*arg == ' ')
		arg++;

	/* Execute specific command */
	enum ftp_cmd c = (enum ftp_cmd)(cmdp-commands);
	switch (c)
	{
	case USER_CMD:
#if defined(_FTP_DEBUG_)
		printf("USER_CMD : %s", arg);
#endif
		slen = getArg(arg);
		strcpy(ftp.username, arg);
		// fsprintf(CTRL_SOCK_CPM, givepass);
		slen = sprintf(sendbuf, "331 Enter PASS command\r\n");
		ret = send(CTRL_SOCK_CPM, (uint8_t *)sendbuf, slen);
		if (ret < 0)
		{
#if defined(_FTP_DEBUG_)
			printf("%d:send() error:%ld\r\n", (int)CTRL_SOCK_CPM, ret);
#endif
			close(CTRL_SOCK_CPM);
			return ret;
		}
		break;

	case PASS_CMD:
#if defined(_FTP_DEBUG_)
		printf("PASS_CMD : %s", arg);
#endif
		slen = getArg(arg);
		ftplogin_cpm(arg);
		break;

	case TYPE_CMD:
		slen = getArg(arg);
		switch (arg[0])
		{
		case 'A':
		case 'a': /* Ascii */
			ftp.type = ASCII_TYPE;
			// fsprintf(CTRL_SOCK_CPM, typeok, arg);
			slen = sprintf(sendbuf, "200 Type set to %s\r\n", arg);
			send(CTRL_SOCK_CPM, (uint8_t*)sendbuf, slen);
			break;

		case 'B':
		case 'b': /* Binary */
		case 'I':
		case 'i': /* Image */
			ftp.type = IMAGE_TYPE;
			// fsprintf(CTRL_SOCK_CPM, typeok, arg);
			slen = sprintf(sendbuf, "200 Type set to %s\r\n", arg);
			send(CTRL_SOCK_CPM, (uint8_t*)sendbuf, slen);
			break;

		default: /* Invalid */
			// fsprintf(CTRL_SOCK_CPM, badtype, arg);
			slen = sprintf(sendbuf, "501 Unknown type \"%s\"\r\n", arg);
			send(CTRL_SOCK_CPM, (uint8_t*)sendbuf, slen);
			break;
		}
		break;

	case FEAT_CMD:
		slen = sprintf(sendbuf, "211-Features:\r\n MDTM\r\n REST STREAM\r\n SIZE\r\n MLST size*;type*;create*;modify*;\r\n MLSD\r\n UTF8\r\n CLNT\r\n MFMT\r\n211 END\r\n");
		send(CTRL_SOCK_CPM, (uint8_t*)sendbuf, slen);
		break;

	case QUIT_CMD:
#if defined(_FTP_DEBUG_)
		printf("QUIT_CMD\r\n");
#endif
		// fsprintf(CTRL_SOCK_CPM, bye);
		slen = sprintf(sendbuf, "221 Goodbye!\r\n");
		send(CTRL_SOCK_CPM, (uint8_t*)sendbuf, slen);
		close(CTRL_SOCK_CPM);
		break;

	case RETR_CMD:
		slen = getArg(arg);
#if defined(_FTP_DEBUG_)
		printf("RETR_CMD\r\n");
#endif
		if (strlen(ftp.workingdir) == 1)
			sprintf(ftp.filename, "/%s", arg);
		else
			sprintf(ftp.filename, "%s/%s", ftp.workingdir, arg);
		slen = sprintf(sendbuf, "150 Opening data channel for file download from server of \"%s\"\r\n", ftp.filename);
		send(CTRL_SOCK_CPM, (uint8_t*)sendbuf, slen);
		ftp.current_cmd = RETR_CMD;
		break;

	case RNFR_CMD:
	{
		mount_drive();
		set_fullpath_cpm(arg);
		struct cpmInode ino;
		char cpmname[2 + 8 + 1 + 3 + 1]; /* 00foobarxy.zzy\0 */
		toCpmName(cpmname, buf);
		int res = cpmNamei(&root, cpmname, &ino);
		struct cpmFile file;
		if (res == -1 || cpmOpen(&ino, &file, O_RDONLY) != 0)
		{
			slen = sprintf(sendbuf, "550 File does not exist\r\n");
		}
		else
		{
			slen = sprintf(sendbuf, "350 File exists, ready for destination name.\r\n");
			strcpy(ftp.filename, buf);
		}
		send(CTRL_SOCK_CPM, (uint8_t*)sendbuf, slen);
		ftp.current_cmd = RNFR_CMD;
	}
		break;

	case RNTO_CMD:
	{
		mount_drive();
		char cpmnameFrom[2 + 8 + 1 + 3 + 1]; /* 00foobarxy.zzy\0 */
		toCpmName(cpmnameFrom, ftp.filename);
		set_fullpath_cpm(arg);
		char cpmnameTo[2 + 8 + 1 + 3 + 1]; /* 00foobarxy.zzy\0 */
		toCpmName(cpmnameTo, buf);
		ftp.fr = cpmRename(&root, cpmnameFrom, cpmnameTo);
		if (ftp.fr != 0)
		{
			slen = sprintf(sendbuf, "550 Unknown error.\r\n");
		}
		else
		{
			slen = sprintf(sendbuf, "250 File or directory renamed successfully.\r\n");
		}
		send(CTRL_SOCK_CPM, (uint8_t*)sendbuf, slen);
		ftp.current_cmd = RNTO_CMD;
	}
	break;

	case APPE_CMD:
	case STOR_CMD:
		slen = getArg(arg);
#if defined(_FTP_DEBUG_)
		printf("STOR_CMD\r\n");
#endif
		if (strlen(ftp.workingdir) == 1)
			sprintf(ftp.filename, /*"/"*/ "%s", arg);
		else
			sprintf(ftp.filename, "%s/%s", ftp.workingdir, arg);
		slen = sprintf(sendbuf, "150 Opening data channel for file upload to server of \"%s\"\r\n", ftp.filename);
		send(CTRL_SOCK_CPM, (uint8_t*)sendbuf, slen);
		ftp.current_cmd = STOR_CMD;
		connect_state_data = 0;
		break;

	case PORT_CMD:
#if defined(_FTP_DEBUG_)
		printf("PORT_CMD\r\n");
#endif
		if (pport_cpm(arg) == -1)
		{
			// fsprintf(CTRL_SOCK_CPM, badport);
			slen = sprintf(sendbuf, "501 Bad port syntax\r\n");
			send(CTRL_SOCK_CPM, (uint8_t*)sendbuf, slen);
		}
		else
		{
			// fsprintf(CTRL_SOCK_CPM, portok);
			ftp.dsock_mode = ACTIVE_MODE;
			ftp.dsock_state = DATASOCK_READY;
			slen = sprintf(sendbuf, "200 PORT command successful.\r\n");
			send(CTRL_SOCK_CPM, (uint8_t*)sendbuf, slen);
		}
		break;

	case MLSD_CMD:
#if defined(_FTP_DEBUG_)
		printf("MLSD_CMD\r\n");
#endif
		slen = sprintf(sendbuf, "150 Opening data channel for directory listing of \"%s\"\r\n", ftp.workingdir);
		send(CTRL_SOCK_CPM, (uint8_t*)sendbuf, slen);
		ftp.current_cmd = MLSD_CMD;
		break;

	case LIST_CMD:
#if defined(_FTP_DEBUG_)
		printf("LIST_CMD\r\n");
#endif
		getArg(arg);
		strcpy(ftp.filename, arg);
		slen = sprintf(sendbuf, "150 Opening data channel for directory listing of \"%s\"\r\n", ftp.workingdir);
		send(CTRL_SOCK_CPM, (uint8_t*)sendbuf, slen);
		ftp.current_cmd = LIST_CMD;
		break;

	case NLST_CMD:
#if defined(_FTP_DEBUG_)
		printf("NLST_CMD\r\n");
#endif
		getArg(arg);
		strcpy(ftp.filename, arg);
		slen = sprintf(sendbuf, "150 Opening data channel for directory listing of \"%s\"\r\n", ftp.workingdir);
		send(CTRL_SOCK_CPM, (uint8_t*)sendbuf, slen);
		ftp.current_cmd = NLST_CMD;
		break;

	case SYST_CMD:
		slen = sprintf(sendbuf, "215 UNIX emulated by WIZnet\r\n");
		send(CTRL_SOCK_CPM, (uint8_t*)sendbuf, slen);
		break;

	case PWD_CMD:
	case XPWD_CMD:
		slen = sprintf(sendbuf, "257 \"%s\" is current directory.\r\n", ftp.workingdir);
		send(CTRL_SOCK_CPM, (uint8_t*)sendbuf, slen);
		break;

	case PASV_CMD:
		slen = sprintf(sendbuf, "227 Entering Passive Mode (%d,%d,%d,%d,%d,%d)\r\n", local_ip.cVal[0], local_ip.cVal[1], local_ip.cVal[2], local_ip.cVal[3], /*(int)(signed char)*/ local_port >> 8, /*(int)(signed char)*/(local_port & 0x00ff));
		send(CTRL_SOCK_CPM, (uint8_t*)sendbuf, slen);
		close(DATA_SOCK_CPM);
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
			//tmpstr = strrchr(arg, '/');
			//*tmpstr = 0;

			if (strlen(ftp.workingdir) == 1)
				sprintf(ftp.filename, "/%s", arg);
			else
				sprintf(ftp.filename, "%s/%s", ftp.workingdir, arg);
			strcpy(buf, ftp.filename[0] == '/' ? ftp.filename + 1 : ftp.filename);
			slen = get_filesize_cpm("", arg/* buf */);
			if (slen > 0)
				slen = sprintf(sendbuf, "213 %d\r\n", slen);
			else
				slen = sprintf(sendbuf, "550 File not Found\r\n");
		}
		else
		{
			slen = sprintf(sendbuf, "550 File not Found\r\n");
		}
		send(CTRL_SOCK_CPM, (uint8_t*)sendbuf, slen);
		break;

	case CWD_CMD:
	    slen = getArg(arg);
		if (slen > 3)
		{
			// arg[slen - 3] = 0x00;
			char *tmpstr = strrchr(arg, '/');
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
					slen = 0;// get_filesize(ftp.workingdir, arg);
				else
					*tmpstr = 0;
				if (tmpstr != NULL)
					slen = 0;//get_filesize(arg, tmpstr + 1);
				// if (slen == -1)
				// 	slen = 0;
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
				strcpy(ftp.username, arg);
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
		send(CTRL_SOCK_CPM, (uint8_t *)sendbuf, slen);
		break;

	case MKD_CMD:
	case XMKD_CMD:
		getArg(arg);
		if (true)
		{
			slen = sprintf(sendbuf, "550 Can't create directory. \"%s\"\r\n", arg);
		}
		else
		{
			slen = sprintf(sendbuf, "257 MKD command successful. \"%s\"\r\n", arg);
			// strcpy(ftp.workingdir, arg);
		}
		send(CTRL_SOCK_CPM, (uint8_t*)sendbuf, slen);
		break;

	case DELE_CMD:
	{
		mount_drive();
		set_fullpath_cpm(arg);
		char cpmname[2 + 8 + 1 + 3 + 1]; /* 00foobarxy.zzy\0 */
		toCpmName(cpmname, buf);
		if (cpmUnlink(&root, cpmname) != 0) // f_unlink(arg) != 0)
		{
			slen = sprintf(sendbuf, "550 Could not delete. \"%s\"\r\n", arg);
		}
		else
		{
			slen = sprintf(sendbuf, "250 Deleted. \"%s\"\r\n", arg);
		}
		send(CTRL_SOCK_CPM, (uint8_t *)sendbuf, slen);
	}
	break;

	case XCWD_CMD:
	case ACCT_CMD:
	case XRMD_CMD:
	case RMD_CMD:
	case STRU_CMD:
	case MODE_CMD:
	case XMD5_CMD:
		// fsprintf(CTRL_SOCK_CPM, unimp);
		slen = sprintf(sendbuf, "502 Command does not implemented yet.\r\n");
		send(CTRL_SOCK_CPM, (uint8_t*)sendbuf, slen);
		break;

	default: /* Invalid */
		// fsprintf(CTRL_SOCK_CPM, badcmd, arg);
		slen = sprintf(sendbuf, "500 Unknown command \'%s\'\r\n", arg);
		send(CTRL_SOCK_CPM, (uint8_t*)sendbuf, slen);
		break;
	}

	return 1;
}

char ftplogin_cpm(char* pass)
{
	char sendbuf[100];
	int slen = 0;

	// memset(sendbuf, 0, DATA_BUF_SIZE);

#if defined(_FTP_DEBUG_)
	printf("%s logged in\r\n", ftp.username);
#endif
	// fsprintf(CTRL_SOCK_CPM, logged);
	slen = sprintf(sendbuf, "230 Logged on\r\n");
	send(CTRL_SOCK_CPM, (uint8_t*)sendbuf, slen);
	ftp.state = FTPS_LOGIN;

	return 1;
}

int pport_cpm(char* arg)
{
	int i;
	char* tok = 0;

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

void print_filedsc_cpm(FIL *fil)
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
