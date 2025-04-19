// SD Controller for Computer "Radio 86RK" / "Apogee BK01"
// (c) 10-05-2014 vinxru (aleksey.f.morozov@gmail.com)

#ifndef COMMON_H
#define COMMON_H

#include <pico.h>

#define CONST
#define USE_DMA 1

enum
{
    SPI_RX  = 12,
    SPI_CSn = 13,
    SPI_SCK = 10,
    SPI_TX  = 11
}; 
#define SPI spi1
#define BAUD (1000*1000)
#define DRQ   4
#define nDACK 5
#define nIOW  6
#define nIOR  7
#define LED   25

#define DRQ_MASK   (1 << DRQ)
#define nDACK_MASK (1 << nDACK)
#define nIOW_MASK  (1 << nIOW)
#define nIOR_MASK  (1 << nIOR)

typedef uint8_t   BYTE;
typedef uint16_t  WORD;
typedef uint32_t  DWORD;

#endif