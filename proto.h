// SD Controller for Computer "Radio 86RK" / "Apogee BK01"
// (c) 10-05-2014 vinxru (aleksey.f.morozov@gmail.com)
#ifndef _PROTO_H
#define _PROTO_H

#include "common.h" 
#include "gpios.h"

#define DATA_OUT()    gpio_set_dir_out_masked(GPIO_CD_MASK)
#define DATA_IN()     gpio_set_dir_in_masked(GPIO_CD_MASK)

#ifndef USE_DMA
void wait();
#endif
void sendStart(BYTE c); 
void send(BYTE c);
void recvStart();
BYTE wrecv();
#ifdef USE_DMA
BYTE dma_receive(BYTE* ptr1, WORD len);
BYTE dma_send(BYTE* ptr1, WORD len);
void sendFlush();
void recvStartNoDma();
#endif

#endif
