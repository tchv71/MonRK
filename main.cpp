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
#include <hardware/spi.h>
#include <common.h>
#include "fifo.pio.h"


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

extern "C" void main_sd();

extern "C" const uint dmaWriteSm;
extern const uint dmaReadSm;
extern "C" const uint fifoWrite2Sm;


extern void __not_in_flash_func(pio_irq_handler_write)();


/*
 * Set up PIOs for pico <-> CPU interface
 */
void dmaPioInit()
{
    uint dmaWriteProgram = pio_add_program(DMA_PIO, &dmaWrite_program);

    pio_sm_config writeConfig = dmaWrite_program_get_default_config(dmaWriteProgram);
    sm_config_set_jmp_pin(&writeConfig, nDACK);
    sm_config_set_in_pins(&writeConfig, GPIO_CD7);
    sm_config_set_in_shift(&writeConfig, false, true, 16); // L shift, autopush @ 16 bits
    sm_config_set_clkdiv(&writeConfig, 1.0f);

    pio_sm_init(DMA_PIO, dmaWriteSm, dmaWriteProgram, &writeConfig);
    pio_sm_set_enabled(DMA_PIO, dmaWriteSm, true/* false */);


#if 1
    int dmaReadProgOffset = pio_add_program(FIFO_PIO, &dmaRead_program);
    if (dmaReadProgOffset<0)
        panic("Failed add dmaReadProgram");

    pio_sm_config readDmaConfig = dmaRead_program_get_default_config(dmaReadProgOffset);
    //sm_config_set_in_pins(&readDmaConfig, GPIO_CSR);
    sm_config_set_jmp_pin(&readDmaConfig, nDACK);
    sm_config_set_sideset_pin_base(&readDmaConfig, DIR);
    sm_config_set_out_pins(&readDmaConfig, GPIO_CD7, 8);
    sm_config_set_in_shift(&readDmaConfig, true, false, 32); // R shift
    sm_config_set_out_shift(&readDmaConfig, true, false, 32); // R shift
    sm_config_set_clkdiv(&readDmaConfig, 1.0f);

    pio_sm_init(FIFO_PIO, dmaReadSm, dmaReadProgOffset, &readDmaConfig);
    pio_sm_set_enabled(FIFO_PIO, dmaReadSm, true);
#endif
}

void setup1()
{    
    spi_init (SPI, BAUD);
    spi_set_format(SPI, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(SPI_RX, GPIO_FUNC_SPI);
    //gpio_set_function(SPI_CSn, GPIO_FUNC_SPI);
    gpio_set_function(SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(SPI_TX, GPIO_FUNC_SPI);

    gpio_init (SPI_CSn);
    gpio_put (SPI_CSn, 1);
    gpio_set_dir (SPI_CSn, GPIO_OUT);
 
    gpio_init (DRQ);
    gpio_put (DRQ, 0);
    gpio_set_dir (DRQ, GPIO_OUT);

    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, 0);

    // SD cards' DO MUST be pulled up.
    gpio_pull_up(SPI_RX);

    dmaPioInit();
    main_sd(); 
}

void loop1()
{

}

void main1()
{
    // rp2040.fifo.registerCore();
    if (setup1)
    {
        setup1();
    }
    while (true)
    {
        if (loop1)
        {
            loop1();
        }
    }
}

extern SerialUSB serial;
/* file globals */

//extern void  __not_in_flash_func(pio_irq_handler)();
//const uint fifoWriteSm = 1;

extern void fifoPioInit();

//uint8_t stream[100];

void setup()
{
    gpio_init_mask(/* GPIO_CD_MASK | */ GPIO_CSW_MASK | GPIO_CSR_MASK | GPIO_A0_MASK);
    fifoPioInit();
    serial.ignoreFlowControl();
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
    multicore_launch_core1(main1);
    //while (true) ;
    setup();
    while (true)
    {
        tight_loop_contents();
        loop();
        __loop();
    }
}
