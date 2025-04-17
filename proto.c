// SD Controller for Computer "Radio 86RK" / "Apogee BK01"
// (c) 10-05-2014 vinxru (aleksey.f.morozov@gmail.com)

#include "proto.h"

#ifndef USE_DMA
void wait()
{
    // ∆дем перепад 0->1
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
BYTE cmd_buf[32];
BYTE* cmd_buf_ptr;
#endif

void sendStart(BYTE c)
{
#ifndef USE_DMA
  wait ();
  DATA_BUS_OUT();
  WRITE_DATA(c);
#else
  dm_mode = DM_SEND;
  cmd_buf_ptr = cmd_buf;
  *cmd_buf_ptr++=c;
#endif
}

#ifdef USE_DMA
void recvStartNoDma()
{
  if (dm_mode == DM_SEND && cmd_buf_ptr != cmd_buf)
    sendFlush();
  dm_mode = DM_RECEIVE;
  cmd_buf_ptr = cmd_buf;
}
#endif

void recvStart()
{
#ifndef USE_DMA
  wait ();
  DATA_BUS_IN ();
#else
  BYTE len[2];
  WORD l;
  recvStartNoDma();
  dma_receive(len, 2);
  l = (len[1] << 8) + len[0];
  dma_receive(cmd_buf, l);
#endif
}

BYTE wrecv()
{
#ifndef USE_DMA
  wait ();
  return READ_DATA();
#else
  return *cmd_buf_ptr++;
#endif
}

void send(BYTE c)
{
#ifndef USE_DMA
  wait ();
  WRITE_DATA(c);
#else
  *cmd_buf_ptr++=c;
#endif
}
#ifdef USE_DMA
void sendFlush()
{
  WORD l = cmd_buf_ptr - cmd_buf;
  BYTE len[2];
  len[0] = l&255;
  len[1] = l>>8;
  if (l == 0)
    return;
  dma_send(len, 2);
  dma_send(cmd_buf, l);
  cmd_buf_ptr = cmd_buf;
}
#endif
