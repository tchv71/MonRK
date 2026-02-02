// SD Controller for Computer "Radio 86RK" / "Apogee BK01"
// (c) 10-05-2014 vinxru (aleksey.f.morozov@gmail.com)

#ifndef COMMON_H
#define COMMON_H

#include <stdint-gcc.h>//<pico.h>
#include <pico/types.h>
#include <sys/cdefs.h>
#include <pico/platform/sections.h>
#include <pico/sync.h>

#define CONST
#define USE_DMA 1
//#define KBD_EMU 1
#define USE_ETHERNET 1
#define USE_SERIAL_DEBUG 1
#ifdef KBD_EMU
#undef USE_SERIAL_DEBUG
#endif
typedef uint8_t   BYTE;
typedef uint16_t  WORD;
typedef uint32_t  DWORD;

#ifdef __cplusplus
extern "C" {
#endif

extern recursive_mutex_t sd_mutex2;
#define get_sd_mutex() &sd_mutex2
#define MTX_ENTER() recursive_mutex_enter_blocking(&sd_mutex2)
#define MTX_TRY_ENTER() recursive_mutex_try_enter(&sd_mutex2, NULL)
#define MTX_EXIT() recursive_mutex_exit(&sd_mutex2)
extern volatile uint8_t v55_buf[4];
#ifdef __cplusplus
}
#endif
#endif