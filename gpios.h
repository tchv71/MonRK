#ifndef _GPIOS_H
#define _GPIOS_H

#define PIN_CD7  14
#define PIN_CSR  26
#define PIN_CSW  27
#define PIN_A0_28   28
#define PIN_A1   29

#define GPIO_CD_MASK (0xff << PIN_CD7)
#define GPIO_CSR_MASK (0x01 << PIN_CSR)
#define GPIO_CSW_MASK (0x01 << PIN_CSW)
#define GPIO_A0_MASK (0x01 << PIN_A0_28)
#define GPIO_A1_MASK (0x01 << PIN_A1)

#define _SPI spi1
#define BAUD (1000*1000)
//#define PIN_nWAIT 2
#define PIN_DRQ   4
#define PIN_nDACK 5
#define PIN_nIOW  0
#define PIN_nIOR  3
#define PIN_nFFFF_R 23
#define PIN_nFFFF_W 2
#define PIN_PDC_nIOR  23
#define PIN_PDC_nIOW  2
#define PIN_HOLD2 9
#define PIN_HLDA 22
#define PIN_ADDRWR 5
#define PIN_DIR   8 // 0 - out, 1 - in
#define PIN_LED   25

#define PDC_nIOR_MASK (1 << PIN_PDC_nIOR)
#define PDC_nIOW_MASK (1 << PIN_PDC_nIOW)
#define HOLD2_MASK   (1 << PIN_HOLD2)
#define HLDA_MASK    (1 << PIN_HLDA)
#define ADDRWR_MASK  (1 << PIN_ADDRWR)

#define PIN_A0 28
#define PIN_A1 29
#define PIN_nCS2 23
#define PIN_nWR 6
#define PIN_nRD 7
#define PIN_USR_KEY 24

enum
{
    PIN_SPI_RX  = 12,
    PIN_SPI_CSn = 13,
    PIN_SPI_SCK = 10,
    PIN_SPI_TX  = 11
}; 


#define A0_MASK   (1 << PIN_A0)
#define A1_MASK   (1 << PIN_A1)
#define nCS2_MASK (1 << PIN_nCS2)
#define nWR_MASK  (1 << PIN_nWR)
#define nRD_MASK  (1 << PIN_nRD)

#define nWAIT_MASK (1 << PIN_nWAIT)
#define DRQ_MASK   (1 << PIN_DRQ)
#define nDACK_MASK (1 << PIN_nDACK)
#define nIOW_MASK  (1 << PIN_nIOW)
#define nIOR_MASK  (1 << PIN_nIOR)
#define DIR_MASK   (1 << PIN_DIR)
#define USR_KEY_MASK (1 << PIN_USR_KEY)


#define FIFO_PIO pio0
#define DMA_PIO pio1
#define FIFO_IRQ PIO0_IRQ_0

#endif
