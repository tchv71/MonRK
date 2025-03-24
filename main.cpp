#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"

// #include "build/pio_programs.pio.h"
#include "pico/multicore.h"
#include "sys/reent.h"
#include <cstdlib>
#include <cstring>
#include "tusb.h"
#include "RP2040USB.h"
#include "Serial.h"
#include "gpios.h"
#include <hardware/vreg.h>
#include <hardware/clocks.h>

#if 0
extern void setup1() __attribute__((weak));
extern void loop1() __attribute__((weak));
#endif
extern void setup();
extern void loop();

// void process_menu(char &inbyte);

namespace arduino
{
    extern void serialEventRun();
    extern void serialEvent1Run();
    extern void serialEvent2Run();
};

// #define USE_TINYUSB 1

// void save_and_reboot();
//--------------------------------------------------------------------+
//  Device
//--------------------------------------------------------------------+

void yield()
{
#ifdef USE_TINYUSB
    TinyUSB_Device_Task();
    TinyUSB_Device_FlushCDC();
#endif
}

extern void __loop()
{
#ifdef USE_TINYUSB
    yield();
#endif

    if (arduino::serialEventRun)
    {
        arduino::serialEventRun();
    }
#if 0
    if (arduino::serialEvent1Run) {
        arduino::serialEvent1Run();
    }
    if (arduino::serialEvent2Run) {
        arduino::serialEvent2Run();
    }
#endif
}
static struct _reent *_impure_ptr1 = nullptr;

#if 0
void main1() 
{
    //rp2040.fifo.registerCore();
    if (setup1) {
        setup1();
    }
    while (true) {
        if (loop1) {
            loop1();
        }
    }
}
#endif

extern SerialUSB serial;
/* file globals */

extern void  __not_in_flash_func(pio_irq_handler)();
const uint fifoWriteSm = 0;
const uint fifoReadSm = 1;

extern void fifoPioInit();

//uint8_t stream[100];

void setup()
{
    gpio_init_mask(GPIO_CD_MASK | GPIO_CSW_MASK | GPIO_CSR_MASK | GPIO_A0_MASK);
    fifoPioInit();
    serial.ignoreFlowControl();
#if 0
    while (!serial.available()) ;
    uint8_t *p = stream;
    while (serial.available())
    {
        uint8_t c = *p++ = serial.read();
        serial.write(c);
    }
    while (true) ;
#endif
}

#define PICO_CLOCK_PLL 1260000000
#define PICO_CLOCK_PLL_DIV1 5
#define PICO_CLOCK_PLL_DIV2 1

int main()
{
    vreg_set_voltage(VREG_VOLTAGE_1_25);
    set_sys_clock_pll(PICO_CLOCK_PLL, PICO_CLOCK_PLL_DIV1, PICO_CLOCK_PLL_DIV2); // 252000

    stdio_init_all();
    // Allocate impure_ptr (newlib temps) if there is a 2nd core running
    // Let rest of core know if we're using FreeRTOS
    bool __isFreeRTOS = false;
#if 0
    if (!__isFreeRTOS && (setup1 || loop1)) {
        _impure_ptr1 = (struct _reent*)calloc(1, sizeof(struct _reent));
        _REENT_INIT_PTR(_impure_ptr1);
    }
#endif
#ifndef NO_USB
#ifdef USE_TINYUSB
    TinyUSB_Device_Init(0);

#else
    __USBStart();

#ifndef DISABLE_USB_SERIAL

    if (!__isFreeRTOS)
    {
        // Enable serial port for reset/upload always
        serial.begin(115200);
    }
#endif
#endif
#endif
    // multicore_launch_core1(main1);
    setup();
    while (true)
    {
        tight_loop_contents();
        loop();
        __loop();
    }
}
