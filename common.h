// SD Controller for Computer "Radio 86RK" / "Apogee BK01"
// (c) 10-05-2014 vinxru (aleksey.f.morozov@gmail.com)

#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>//<pico.h>
#include <pico/types.h>
#include <sys/cdefs.h>
#include <pico/platform/sections.h>
#include <pico/sync.h>

#define CONST
#define USE_DMA 0
#define USE_ETHERNET 0
#define USE_SERIAL_DEBUG 1

typedef uint8_t   BYTE;
typedef uint16_t  WORD;
typedef uint32_t  DWORD;

#ifdef __cplusplus
extern "C" {
#endif
mutex_t* get_sd_mutex();
extern uint8_t v55_buf[4];
#ifdef __cplusplus
}
#endif
#endif