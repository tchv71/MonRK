// SD Controller for Computer "Radio 86RK" / "Apogee BK01"
// (c) 10-05-2014 vinxru (aleksey.f.morozov@gmail.com)

#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>//<pico.h>
#include <pico/types.h>
#include <sys/cdefs.h>
#include <pico/platform/sections.h>
#include <pico/sync.h>
#include <lwip/ip_addr.h>

#define CONST
#define USE_DMA 0//1
#define USE_ETHERNET 1
#define USE_SERIAL_DEBUG 1

enum
{
    SPI_RX  = 12,
    SPI_CSn = 13,
    SPI_SCK = 10,
    SPI_TX  = 11
}; 
#define _SPI spi1
#define BAUD (1000*1000)
#define PIN_DRQ   4
#define PIN_nDACK 5
#define PIN_nIOW  6
#define PIN_nIOR  7
#define PIN_DIR   8
#define PIN_LED   25

#define PIN_A0 0
#define PIN_A1 1
#define PIN_nCS2 2
#define PIN_nWR PIN_nIOW
#define PIN_nRD PIN_nIOR

#define A0_MASK   (1 << PIN_A0)
#define A1_MASK   (1 << PIN_A1)
#define nCS2_MASK (1 << PIN_nCS2)
#define nWR_MASK  (1 << PIN_nWR)
#define nRD_MASK  (1 << PIN_nRD)

#define DRQ_MASK   (1 << PIN_DRQ)
#define nDACK_MASK (1 << PIN_nDACK)
#define nIOW_MASK  (1 << PIN_nIOW)
#define nIOR_MASK  (1 << PIN_nIOR)
#define DIR_MASK   (1 << PIN_DIR)

typedef uint8_t   BYTE;
typedef uint16_t  WORD;
typedef uint32_t  DWORD;

typedef struct TCP_SERVER_T_
{
    struct tcp_pcb *server_pcb;
    struct tcp_pcb *client_pcb;
    ip_addr_t remote_addr;
    uint16_t remote_port;
    bool complete;
    //uint8_t buffer_sent[BUF_SIZE];
    //uint8_t buffer_recv[BUF_SIZE];
    int sent_len;
    int recv_len;
    int run_count;
    uint8_t sock_state;
} TCP_SERVER_T;


#ifdef __cplusplus
extern "C" {
#endif
    mutex_t *get_sd_mutex();
#ifdef __cplusplus
}
#endif
#endif