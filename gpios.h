#ifndef _GPIOS_H
#define _GPIOS_H

#define PIN_CD7  14
#define PIN_CSR  26
#define PIN_CSW  27
#define PIN_A0_28   28

#define GPIO_CD_MASK (0xff << PIN_CD7)
#define GPIO_CSR_MASK (0x01 << PIN_CSR)
#define GPIO_CSW_MASK (0x01 << PIN_CSW)
#define GPIO_A0_MASK (0x01 << PIN_A0_28)

#define FIFO_PIO pio0
#define DMA_PIO pio1
#define FIFO_IRQ PIO0_IRQ_0

extern const uint fifoWriteSm;
extern const uint fifoReadSm;
extern const uint dmaReadSm;

#endif
