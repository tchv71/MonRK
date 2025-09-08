#ifndef _FTPD_CPM_H_
#define _FTPD_CPM_H_
#include "ftpd.h"

#ifdef __cplusplus
extern "C" {
#endif
	/* Buffer */
#define ETHERNET_BUF_MAX_SIZE (1024 * 16)

/*
* Wiznet.
* (c) Copyright 2002, Wiznet.
*
* Filename	: ftpd.h
* Version	: 1.0
* Programmer(s)	: 
* Created	: 2003/01/28
* Description   : Header file of FTP daemon. (AVR-GCC Compiler)
*/

#include <stdint.h>

#define F_FILESYSTEM_CPM // If your target support a file system, you have to activate this feature and implement.

#if defined(F_FILESYSTEM_CPM)
//#include "ff.h"
#include "../../../../cpmfs.h"
#endif

#define F_APP_FTP
//#define _FTP_DEBUG_


#define LINELEN		100
//#define DATA_BUF_SIZE	100
#if !defined(F_FILESYSTEM_CPM)
#define _MAX_SS		512
#endif

#define CTRL_SOCK_CPM	4
#define DATA_SOCK_CPM	5

#define	IPPORT_FTPD	20	/* FTP Data port */
#define	IPPORT_FTP	21	/* FTP Control port */

#define HOSTNAME	"iinChip"
#define VERSION		"1.0"


void ftpd_cpm_init(uint8_t * src_ip);
long ftpd_cpm_run(uint8_t *dbuf);
void toCpmName(char cpmname[15], const char* filename);
long proc_ftpd_cpm(char * buf);
char ftplogin_cpm(char * pass);
int pport_cpm(char * arg);

int sendit_cpm(char * command);
int recvi_cpm(char * command);

long sendfile_cpm(uint8_t s, char * command);
long recvfile_cpm(uint8_t s);

#if defined(F_FILESYSTEM_CPM)
void print_filedsc_cpm(FIL *fil);
#endif

#ifdef __cplusplus
}
#endif

#endif // _FTPD_CPM_H_
