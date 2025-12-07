// SD Controller for Computer "Radio 86RK" / "Apogee BK01"
// (c) 10-05-2014 vinxru (aleksey.f.morozov@gmail.com)
#ifndef _PROTO_H
#define _PROTO_H

#include "common.h" 
#include "gpios.h"

#define DATA_OUT()    gpio_set_dir_out_masked(GPIO_CD_MASK)
#define DATA_IN()     gpio_set_dir_in_masked(GPIO_CD_MASK)

#if !USE_DMA
extern critical_section_t g_wizchip_cri_sec;
#define WAIT_RW_BYTE()

void  __time_critical_func(wait)();
#endif
void sendStart(BYTE c); 
void sendByte(BYTE c);
void sendWord(WORD w);
#define recvStart()
BYTE recvByte();
WORD recvWord();
#if USE_DMA
void dma_receive(BYTE* ptr, WORD len);
void dma_send(const BYTE* ptr, WORD len);
#define recvStartNoDma()
#endif
/**
  * @brief  RTC Date structure definition
  */
typedef struct
{
  uint8_t WeekDay;  /*!< Specifies the RTC Date WeekDay.
                         This parameter can be a value of @ref RTC_WeekDay_Definitions */

  uint8_t Month;    /*!< Specifies the RTC Date Month (in BCD format).
                         This parameter can be a value of @ref RTC_Month_Date_Definitions */

  uint8_t Date;     /*!< Specifies the RTC Date.
                         This parameter must be a number between Min_Data = 1 and Max_Data = 31 */

  uint8_t Year;     /*!< Specifies the RTC Date Year.
                         This parameter must be a number between Min_Data = 0 and Max_Data = 99 */

} RTC_DateTypeDef;
#endif
