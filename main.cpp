#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"

// #include "build/pio_programs.pio.h"
#include "pico/multicore.h"
#include "sys/reent.h"
#include <cstdlib>
#include <cstring>
//#include "tusb.h"
//#include "RP2040USB.h"
//#include "Serial.h"
#include "gpios.h"
#include <hardware/vreg.h>
#include <hardware/clocks.h>
#include <hardware/spi.h>
#include <common.h>
#include "fifo.pio.h"
#include "pico/sync.h"



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
#if 0
#ifdef USE_TINYUSB
    yield();
#endif

    tud_task(); // tinyusb device task
    if (false && arduino::serialEventRun)
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
 * FIFO pio for read-like programs
 * DMA pio for write-like due to the same used pins
 */
void dmaPioInit()
{
    // DMA Read  = SD  -> Mem
    // DMA Write = Mem -> SD
    uint dmaWriteProgram = pio_add_program(DMA_PIO, &dmaWrite_program);

    pio_sm_config writeConfig = dmaWrite_program_get_default_config(dmaWriteProgram);
    sm_config_set_jmp_pin(&writeConfig, nDACK);
    sm_config_set_in_pins(&writeConfig, GPIO_CD7);
    sm_config_set_in_shift(&writeConfig, false, true, 16); // L shift, autopush @ 16 bits
    sm_config_set_clkdiv(&writeConfig, 1.0f);

    pio_sm_init(DMA_PIO, dmaWriteSm, dmaWriteProgram, &writeConfig);
    pio_sm_set_enabled(DMA_PIO, dmaWriteSm, true/* false */);


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
}

void setup1()
{    
    spi_init (_SPI, BAUD);
    spi_set_format(_SPI, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(SPI_RX, GPIO_FUNC_SPI);
    //gpio_set_function(SPI_CSn, GPIO_FUNC_SPI);
    gpio_set_function(SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(SPI_TX, GPIO_FUNC_SPI);

    gpio_init_mask(A0_MASK | A1_MASK | nCS2_MASK | GPIO_CD_MASK | nWR_MASK | nRD_MASK);

    gpio_init (SPI_CSn);
    gpio_put (SPI_CSn, 1);
    gpio_set_dir (SPI_CSn, GPIO_OUT);
 
    gpio_init (DRQ);
    gpio_put (DRQ, 0);
    gpio_set_dir (DRQ, GPIO_OUT);

    gpio_init(nDACK);
    gpio_set_dir(nDACK, GPIO_IN);

    gpio_init(nIOR);
    gpio_set_dir(nIOR, GPIO_IN);

    gpio_init(nIOW);
    gpio_set_dir(nIOW, GPIO_IN);

    // gpio_init(25);
    // gpio_set_dir(25, GPIO_OUT);
    // gpio_put(25, 1);

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

//extern SerialUSB serial;
/* file globals */

extern void fifoPioInit();
extern void networkInit();

void setup()
{
    gpio_init_mask(GPIO_CD_MASK | GPIO_CSW_MASK | GPIO_CSR_MASK | GPIO_A0_MASK);
    gpio_set_dir_in_masked(GPIO_CSW_MASK | GPIO_CSR_MASK | GPIO_A0_MASK);
    fifoPioInit();
    //serial.ignoreFlowControl();
#if USE_ETHERNET
    networkInit();
#endif
}

#define PICO_CLOCK_PLL 1260000000
#define PICO_CLOCK_PLL_DIV1 5
#define PICO_CLOCK_PLL_DIV2 1
extern TCP_SERVER_T* state;

int main()
{
    vreg_set_voltage(VREG_VOLTAGE_1_30);
    set_sys_clock_pll(PICO_CLOCK_PLL, PICO_CLOCK_PLL_DIV1, PICO_CLOCK_PLL_DIV2); // 252000
    //set_sys_clock_khz(290000, false);
    stdio_init_all();
    //stdio_set_translate_crlf(&stdio_usb, false);

   multicore_launch_core1(main1);
   setup();
    //__USBStart();

    //while (true) ;
    //setup();
    while (true)
    {
        tight_loop_contents();
        loop();
        //_loop();
    }
}
