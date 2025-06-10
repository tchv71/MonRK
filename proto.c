// SD Controller for Computer "Radio 86RK" / "Apogee BK01"
// (c) 10-05-2014 vinxru (aleksey.f.morozov@gmail.com)

#include "proto.h"
#include "pico/time.h"

#ifndef USE_DMA
void wait()
{
    // Ждем перепад 0->1
    while(!PINC.5);
    while(PINC.5); 
    if((PINC&0x3F)==0) return;
#asm 
    RJMP 0
#endasm

}
#endif
#ifdef USE_DMA
enum DMA_MODE { DM_NONE = 0, DM_SEND, DM_RECEIVE} dm_mode;
BYTE cmd_buf_send[32];
BYTE cmd_buf_recv[32];
BYTE* cmd_buf_send_ptr;
BYTE* cmd_buf_recv_ptr;
#endif

void sendStart(BYTE c)
{
#ifndef USE_DMA
  wait ();
  DATA_BUS_OUT();
  WRITE_DATA(c);
#else
  dm_mode = DM_SEND;
  cmd_buf_send_ptr = cmd_buf_send;
  *cmd_buf_send_ptr++=c;
#endif
}

#ifdef USE_DMA
void recvStartNoDma()
{
  if (dm_mode == DM_SEND && cmd_buf_send_ptr != cmd_buf_send)
    sendFlush();
  dm_mode = DM_RECEIVE;
  cmd_buf_recv_ptr = cmd_buf_recv;
}

WORD l;
#endif

void recvStart()
{
#ifndef USE_DMA
  wait ();
  DATA_BUS_IN ();
#else
  recvStartNoDma();
  dma_receive((BYTE*)&l, 2);
  dma_receive(cmd_buf_recv, l);
#endif
}

BYTE wrecv()
{
#ifndef USE_DMA
  wait ();
  return READ_DATA();
#else
  return *cmd_buf_recv_ptr++;
#endif
}

void sendByte(BYTE c)
{
#ifndef USE_DMA
  wait ();
  WRITE_DATA(c);
#else
  *cmd_buf_send_ptr++=c;
#endif
}
#ifdef USE_DMA
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
