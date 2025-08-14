// SD Controller for Computer "Radio 86RK" / "Apogee BK01"
// (c) 10-05-2014 vinxru (aleksey.f.morozov@gmail.com)

#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>//<pico.h>
#include <pico/types.h>
#include <sys/cdefs.h>
#include <pico/platform/sections.h>
#include <pico/sync.h>

#define CONST
#define USE_DMA 1
#define USE_ETHERNET 1

enum
{
    SPI_RX  = 12,
    SPI_CSn = 13,
    SPI_SCK = 10,
    SPI_TX  = 11
}; 
#define _SPI spi1
#define BAUD (1000*1000)
#define DRQ   4
#define nDACK 5
#define nIOW  6
#define nIOR  7
#define DIR   8
#define LED   25

#define DRQ_MASK   (1 << DRQ)
#define nDACK_MASK (1 << nDACK)
#define nIOW_MASK  (1 << nIOW)
#define nIOR_MASK  (1 << nIOR)
#define DIR_MASK   (1 << DIR)

typedef uint8_t   BYTE;
typedef uint16_t  WORD;
typedef uint32_t  DWORD;

#ifdef __cplusplus
extern "C" {
#endif
mutex_t* get_sd_mutex();
#ifdef __cplusplus
}
#endif
#endif