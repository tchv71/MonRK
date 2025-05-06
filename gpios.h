#ifndef _GPIOS_H
#define _GPIOS_H

#define GPIO_CD7  14
#define GPIO_CSR  26
#define GPIO_CSW  27
#define GPIO_A0   28

#define GPIO_CD_MASK (0xff << GPIO_CD7)
#define GPIO_CSR_MASK (0x01 << GPIO_CSR)
#define GPIO_CSW_MASK (0x01 << GPIO_CSW)
#define GPIO_A0_MASK (0x01 << GPIO_A0)

#define FIFO_PIO pio0
#define DMA_PIO pio1
#define FIFO_IRQ PIO0_IRQ_0

extern const uint fifoWriteSm;
extern const uint fifoReadSm;
extern const uint dmaReadSm;

#endif
