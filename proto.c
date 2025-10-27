// SD Controller for Computer "Radio 86RK" / "Apogee BK01"
// (c) 10-05-2014 vinxru (aleksey.f.morozov@gmail.com)

#include "proto.h"
#include "pico/time.h"
#include "hardware/pio.h"

void RkSd_Main();

extern const uint dmaRomSm;


#if !USE_DMA
void DATA_BUS_OUT()
{}

void DATA_BUS_IN()
{}

void __not_in_flash_func(WRITE_DATA)(uint8_t val)
{
  v55_buf[0] = val;
  pio_sm_put(FIFO_PIO, dmaRomSm, 0xFF << 8 | val);
}

uint8_t __not_in_flash_func(READ_DATA)()
{
  return v55_buf[0];
}

#endif

void __not_in_flash_func(sendStart)(BYTE c)
{
#if !USE_DMA
  wait ();
  DATA_BUS_OUT();
  WRITE_DATA(c);
#else
  sendByte(c);
#endif
}

#if USE_DMA
#define recvStartNoDma()

#else
extern volatile bool bEvent;
void  inline __time_critical_func(wait)()
{
#if 1
  //MTX_ENTER();
  // Ждем перепад 1->0 A5
  while ((v55_buf[1] & 0x20) == 0)
    WAIT_RW_BYTE();
  while ((v55_buf[1] & 0x20) != 0)
    WAIT_RW_BYTE();
  //MTX_EXIT();
#else
  do
  {
    __wfe();
  } while (!bEvent);
  bEvent = false;
#endif
}
#endif

#if !USE_DMA
void __not_in_flash_func(recvStart)()
{
  wait ();
  DATA_BUS_IN ();
}
#else
#define recvStart()
#endif

BYTE __not_in_flash_func(wrecv)()
{
#if !USE_DMA
  wait ();
  return READ_DATA();
#else
  BYTE c;
  dma_receive(&c, 1);
   return c;
#endif
}

void __not_in_flash_func(sendByte)(BYTE c)
{
#if !USE_DMA
  wait ();
  WRITE_DATA(c);
#else
  static BYTE c1;
  c1 = c;
  dma_send(&c1, 1);
#endif
}

void __not_in_flash_func(sendWord)(WORD w)
{
#if !USE_DMA
  wait ();
  WRITE_DATA(c);
#else
  static WORD w1;
  w1 = w;
  dma_send((const BYTE*)&w1, 2);
#endif
}