// SD Controller for Computer "Radio 86RK" / "Apogee BK01"
// (c) 10-05-2014 vinxru (aleksey.f.morozov@gmail.com)

#include "proto.h"
#include "pico/time.h"
#include "hardware/pio.h"

void RkSd_Main();

extern const uint dmaRomSm;


#if USE_DMA
enum DMA_MODE { DM_NONE = 0, DM_SEND, DM_RECEIVE} dm_mode;
BYTE cmd_buf_send[32];
BYTE cmd_buf_recv[32];
BYTE* cmd_buf_send_ptr = cmd_buf_send;
BYTE* cmd_buf_recv_ptr = cmd_buf_recv;
#else
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
  dm_mode = DM_SEND;
  cmd_buf_send_ptr = cmd_buf_send;
  *cmd_buf_send_ptr++=c;
#endif
}

#if USE_DMA
void recvStartNoDma()
{
  if (dm_mode == DM_SEND && cmd_buf_send_ptr != cmd_buf_send)
    sendFlush();
  dm_mode = DM_RECEIVE;
  cmd_buf_recv_ptr = cmd_buf_recv;
}

WORD l;
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

void __not_in_flash_func(recvStart)()
{
#if !USE_DMA
  wait ();
  DATA_BUS_IN ();
#else
  recvStartNoDma();
  dma_receive((BYTE*)&l, 2);
  dma_receive(cmd_buf_recv, l);
#endif
}

BYTE __not_in_flash_func(wrecv)()
{
#if !USE_DMA
  wait ();
  return READ_DATA();
#else
  return *cmd_buf_recv_ptr++;
#endif
}

void __not_in_flash_func(sendByte)(BYTE c)
{
#if !USE_DMA
  wait ();
  WRITE_DATA(c);
#else
  *cmd_buf_send_ptr++=c;
#endif
}
#if USE_DMA
WORD lSend;
void sendFlush()
{
  lSend = cmd_buf_send_ptr - cmd_buf_send;
  BYTE len[2];
  len[0] = lSend&255;
  len[1] = lSend>>8;
  if (lSend == 0)
    return;
  dma_send(len, 2);
  dma_send(cmd_buf_send, lSend);
  cmd_buf_send_ptr = cmd_buf_send;
}
#endif
