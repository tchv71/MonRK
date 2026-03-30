// SD Controller for Computer "Radio 86RK" / "Apogee BK01"
// (c) 10-05-2014 vinxru (aleksey.f.morozov@gmail.com)

#ifndef COMMON3_H
#define COMMON3_H
typedef unsigned int uint;

#include <pico.h>
#include <pico/platform.h>
#include <machine/_default_types.h>
#include <sys/_stdint.h>
#include <pico/types.h>
#include <sys/cdefs.h>
#include <pico/platform/sections.h>
#include <hardware/address_mapped.h>
#include <hardware/sync.h>
#include "pico/lock_core.h"
#include <pico/sync.h>
#include <pico/mutex.h>

// #ifdef __cplusplus
// extern "C" {
// #endif

#define CONST
#define USE_DMA 1
#define KBD_EMU 1
//#define DMA_TEST 1
#define USE_ETHERNET 1
#define USE_SERIAL_DEBUG 1
#ifdef KBD_EMU
#undef USE_SERIAL_DEBUG
#endif
typedef unsigned char BYTE;
typedef unsigned short  WORD;
typedef unsigned int  DWORD;


extern recursive_mutex_t sd_mutex2;
#define get_sd_mutex() &sd_mutex2
#define MTX_ENTER() recursive_mutex_enter_blocking(&sd_mutex2)
#define MTX_TRY_ENTER() recursive_mutex_try_enter(&sd_mutex2, NULL)
#define MTX_EXIT() recursive_mutex_exit(&sd_mutex2)

extern bool bKbdEmu;
extern bool lastLedVal;

#define LedUpdate() gpio_put(PIN_LED, lastLedVal^(!bKbdEmu))  
#define Led(value)  lastLedVal = value; LedUpdate()
#define LedOff() Led(false)
#define LedOn()  Led(true)

#define dmaWriteSm 0
#define fifoRomRdSm 2
#define dmaReadSm  1
#define fifoReadSm 0
#define fifoWrite2Sm 1
#define dmaRomWrSm 3

// #ifdef __cplusplus
// }
// #endif
#endif
