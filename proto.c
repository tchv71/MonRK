// SD Controller for Computer "Radio 86RK" / "Apogee BK01"
// (c) 10-05-2014 vinxru (aleksey.f.morozov@gmail.com)

#include "proto.h"
#include "pico/time.h"
#include "hardware/pio.h"

void RkSd_Main();

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

BYTE __not_in_flash_func(recvByte)()
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

WORD __not_in_flash_func(recvWord)()
{
#if !USE_DMA
  WORD w = wrecv();
  return w | ((WORD)(wrecv()))<<8;
#else
  WORD w;
  dma_receive((BYTE*)&w, 2);
  return w;
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


#define INTS_OFF 0
void __not_in_flash_func(dma_send)(const BYTE *ptr, WORD len)
{
#if 1
  gpio_put(PIN_DRQ, 1);
#if INTS_OFF
  uint32_t ints = save_and_disable_interrupts();
#endif
  uint32_t val;
  do
  {
    val = *ptr++ | (0xFF << 8);
    pio_sm_put_blocking(FIFO_PIO, dmaReadSm, val);
  } while (--len);
#if INTS_OFF
  restore_interrupts(ints);
#endif
  while (!pio_sm_is_tx_fifo_empty(FIFO_PIO, dmaReadSm)) ;
  while (gpio_get(PIN_nDACK) == 0) ;
  gpio_put(PIN_DRQ, 0);
#else
  gpio_init(PIN_DRQ);
  gpio_put(PIN_DRQ, 1);
  gpio_set_dir(PIN_DRQ, GPIO_OUT);
  uint32_t ints = save_and_disable_interrupts();
  do
  {
    DATA_IN();
    while (gpio_get_all() & (nIOR_MASK | nDACK_MASK))
      ;
    DATA_OUT();
    WRITE_DATA(*ptr++); // PORTD = *ptr++;
    while (gpio_get(nIOR) == 0)
      ;
  } while (--len);
  gpio_put(PIN_DRQ, 0);
  restore_interrupts(ints);
#endif
}

void __not_in_flash_func(dma_receive)(BYTE *ptr, WORD len)
{
#if 1
  gpio_put(PIN_DRQ, 1);
#if INTS_OFF
  uint32_t ints = save_and_disable_interrupts();
#endif
  //uint16_t *ptr1 = (uint16_t*)ptr;
  //len /= 2;
  do
  {
    *ptr++ = pio_sm_get_blocking(DMA_PIO, dmaWriteSm) & 0xFF;
  } while (--len);
#if INTS_OFF
  restore_interrupts(ints);
#endif
  gpio_put(PIN_DRQ, 0);
  while (gpio_get(PIN_nWR) == 0) ;
#else
  DATA_IN();
  gpio_put(PIN_DRQ, 1);
  uint32_t ints = save_and_disable_interrupts();
  do
  {
    while (gpio_get_all() & (nIOW_MASK | nDACK_MASK))
      ;
    while (gpio_get(nIOW) == 0)
      ;
    *ptr++ = READ_DATA();
  } while (--len);
  gpio_put(DRQ, 0);
  restore_interrupts(ints);
#endif
}
