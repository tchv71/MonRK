#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"

// #include "build/pio_programs.pio.h"
#include "pico/multicore.h"
#include "sys/reent.h"
#include <cstdlib>
#include <cstring>
// #include "tusb.h"
// #include "RP2040USB.h"
// #include "Serial.h"
#include "gpios.h"
#include <hardware/vreg.h>
#include <hardware/clocks.h>
#include <hardware/spi.h>
#include <common.h>
#include "fifo.pio.h"
#include "pico/sync.h"
#include "CircBuffer.h"

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
extern "C" const uint dmaRomSm;
extern const uint dmaReadSm;
extern "C" const uint fifoWrite2Sm;

extern void __not_in_flash_func(pio_irq_handler_write)();
extern void __not_in_flash_func(pio_irq_handler_rom)();
extern CircBuffer<10 * 1024> bufIn;
extern CircBuffer<1024> bufOut;

/*
 * Set up PIOs for pico <-> CPU interface
 */
void fifoPioInit()
{
    bufIn.clear();
    bufOut.clear();

#if USE_SERIAL_DEBUG
    pio_sm_claim(FIFO_PIO, fifoReadSm);
    int fifoReadProgOffset = pio_add_program(FIFO_PIO, &fifoRead_program);
    if (fifoReadProgOffset < 0)
        panic("Failed add fifoReadProgram");

    for (uint i = 0; i < 8; ++i)
    {
        pio_gpio_init(FIFO_PIO, PIN_CD7 + i);
        gpio_set_drive_strength(PIN_CD7 + i, GPIO_DRIVE_STRENGTH_12MA);
    }
    pio_gpio_init(FIFO_PIO, PIN_DIR);
    pio_sm_set_pins_with_mask(FIFO_PIO, fifoReadSm, DIR_MASK, DIR_MASK);
    pio_sm_set_pindirs_with_mask(FIFO_PIO, fifoReadSm, DIR_MASK, DIR_MASK);
    // pio_sm_set_consecutive_pindirs(FIFO_PIO, fifoReadSm, DIR, 1, true);
    //hw_set_bits(&FIFO_PIO->input_sync_bypass, (uint32_t)-1/* (1u << PIN_A0_28) | (1u << PIN_CSR) */);
    /* fifoReadProg */
    pio_sm_config readFifoConfig = fifoRead_program_get_default_config(fifoReadProgOffset);
    sm_config_set_in_pins(&readFifoConfig, PIN_CSR);
    sm_config_set_jmp_pin(&readFifoConfig, PIN_A0_28);
    sm_config_set_mov_status(&readFifoConfig, STATUS_TX_LESSTHAN, 1);
    sm_config_set_fifo_join(&readFifoConfig, PIO_FIFO_JOIN_TX);
    //sm_config_set_sideset_pin_base(&readFifoConfig, PIN_DIR);
    sm_config_set_out_pins(&readFifoConfig, PIN_CD7, 8);
    sm_config_set_in_shift(&readFifoConfig, true, false, 32);  // R shift
    sm_config_set_out_shift(&readFifoConfig, true, false, 32); // R shift
    sm_config_set_clkdiv(&readFifoConfig, 1.0f);
    //
    pio_sm_init(FIFO_PIO, fifoReadSm, fifoReadProgOffset, &readFifoConfig);
    // pio_set_irq0_source_enabled(FIFO_PIO, pis_sm0_rx_fifo_not_empty, true);
    // irq_set_exclusive_handler(PIO0_IRQ_0, pio_irq_handler_read);
    // irq_set_enabled(PIO0_IRQ_0, true);
    pio_sm_set_enabled(FIFO_PIO, fifoReadSm, true);
    // updateFifoReadAhead();

    /* fifoWriteProg */
    pio_sm_claim(DMA_PIO, fifoWrite2Sm);
    int fifoWriteProgOffset = pio_add_program(DMA_PIO, &fifoWrite_program);
    if (fifoWriteProgOffset < 0)
        panic("Failed add fifoWriteProgram");
    pio_sm_config writeConfig2 = fifoWrite_program_get_default_config(fifoWriteProgOffset);
    sm_config_set_in_pins(&writeConfig2, PIN_CD7);
    sm_config_set_in_shift(&writeConfig2, false, true, 16); // L shift, autopush @ 16 bits
    sm_config_set_clkdiv(&writeConfig2, 1.0f);
    //
    pio_sm_init(DMA_PIO, fifoWrite2Sm, fifoWriteProgOffset, &writeConfig2);
    pio_sm_set_enabled(DMA_PIO, fifoWrite2Sm, true);
    pio_set_irq1_source_enabled(DMA_PIO, pis_sm1_rx_fifo_not_empty, true);
    irq_set_exclusive_handler(PIO1_IRQ_1, pio_irq_handler_write);
    irq_set_enabled(PIO1_IRQ_1, true);
#endif
}
/*
 * Set up PIOs for pico <-> CPU interface
 * FIFO pio for read-like programs
 * DMA pio for write-like due to the same used pins
 */
void dmaPioInit()
{
#if !USE_DMA
    // DMA Read  = SD  -> Mem
    // DMA Write = Mem -> SD
    int romProgramOffset = pio_add_program(FIFO_PIO, &rom_program);
    if (romProgramOffset < 0)
        panic("Failed add fifoReadProgram");
    pio_sm_clear_fifos(FIFO_PIO, dmaRomSm);

    pio_sm_config romConfig = rom_program_get_default_config(romProgramOffset);
    sm_config_set_in_pins(&romConfig, PIN_A0);
    sm_config_set_jmp_pin(&romConfig, PIN_nWR);
    sm_config_set_sideset_pin_base(&romConfig, PIN_DIR);
    sm_config_set_out_pins(&romConfig, PIN_CD7, 8);
#define SH_LEFT false
#define SH_RIGHT true
    sm_config_set_in_shift(&romConfig, SH_LEFT, false, 32);   // L shift
    sm_config_set_out_shift(&romConfig, SH_RIGHT, false, 32); // R shift
    sm_config_set_clkdiv(&romConfig, 1.0f);

    pio_sm_init(FIFO_PIO, dmaRomSm, romProgramOffset, &romConfig);
    pio_set_irq1_source_enabled(FIFO_PIO, pis_sm1_rx_fifo_not_empty, true);
    irq_set_exclusive_handler(PIO0_IRQ_1, pio_irq_handler_rom);
    irq_set_enabled(PIO0_IRQ_1, true);
    pio_sm_set_enabled(FIFO_PIO, dmaRomSm, true /* false */);
    enable_interrupts();
#else
    uint dmaWriteProgOffset = pio_add_program(DMA_PIO, &dmaWrite_program);
     if (dmaWriteProgOffset < 0)
        panic("Failed add dmaWriteProgram");
   
    pio_sm_config writeConfig = dmaWrite_program_get_default_config(dmaWriteProgOffset);
    sm_config_set_jmp_pin(&writeConfig, PIN_nDACK);
    sm_config_set_fifo_join(&writeConfig, PIO_FIFO_JOIN_RX);
    sm_config_set_in_pins(&writeConfig, PIN_CD7);
    sm_config_set_in_shift(&writeConfig, false, true, 16); // L shift, autopush @ 16 bits
    sm_config_set_clkdiv(&writeConfig, 1.0f);

    pio_sm_init(DMA_PIO, dmaWriteSm, dmaWriteProgOffset, &writeConfig);
    pio_sm_set_enabled(DMA_PIO, dmaWriteSm, true/* false */);
    //
    int dmaReadProgOffset = pio_add_program(FIFO_PIO, &dmaRead_program);
    if (dmaReadProgOffset<0)
        panic("Failed add dmaReadProgram");

    pio_gpio_init(FIFO_PIO, PIN_DIR);
    pio_sm_set_pins_with_mask(FIFO_PIO, dmaReadSm, DIR_MASK, DIR_MASK);
    pio_sm_set_pindirs_with_mask(FIFO_PIO, dmaReadSm, DIR_MASK, DIR_MASK);

    pio_sm_config readDmaConfig = dmaRead_program_get_default_config(dmaReadProgOffset);
    //sm_config_set_in_pins(&readDmaConfig, PIN_CSR);
    sm_config_set_jmp_pin(&readDmaConfig, PIN_nDACK);
    sm_config_set_fifo_join(&readDmaConfig, PIO_FIFO_JOIN_TX);
    sm_config_set_sideset_pin_base(&readDmaConfig, PIN_DIR);
    sm_config_set_out_pins(&readDmaConfig, PIN_CD7, 8);
    sm_config_set_in_shift(&readDmaConfig, true, false, 32); // R shift
    sm_config_set_out_shift(&readDmaConfig, true, false, 32); // R shift
    sm_config_set_clkdiv(&readDmaConfig, 1.0f);

    pio_sm_init(FIFO_PIO, dmaReadSm, dmaReadProgOffset, &readDmaConfig);
    pio_sm_set_enabled(FIFO_PIO, dmaReadSm, true);
#endif
}

void setup1()
{
    spi_init(_SPI, BAUD);
    spi_set_format(_SPI, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(PIN_SPI_RX, GPIO_FUNC_SPI);
    // gpio_set_function(PIN_SPI_CSn, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_TX, GPIO_FUNC_SPI);

    gpio_init_mask(A0_MASK | A1_MASK | nCS2_MASK | /* GPIO_CD_MASK |  */ nWR_MASK | nRD_MASK | nIOR_MASK | nIOR_MASK);

    gpio_init(PIN_SPI_CSn);
    gpio_put(PIN_SPI_CSn, 1);
    gpio_set_dir(PIN_SPI_CSn, GPIO_OUT);

    gpio_init(PIN_DRQ);
    gpio_put(PIN_DRQ, 0);
    gpio_set_dir(PIN_DRQ, GPIO_OUT);

    gpio_init(PIN_nDACK);
    gpio_set_dir(PIN_nDACK, GPIO_IN);

    gpio_init(PIN_nIOR);
    gpio_set_dir(PIN_nIOR, GPIO_IN);

    gpio_init(PIN_nIOW);
    gpio_set_dir(PIN_nIOW, GPIO_IN);

    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, 0);

    // SD cards' DO MUST be pulled up.
    gpio_pull_up(PIN_SPI_RX);

    dmaPioInit();
}

void loop1()
{
    main_sd();
}

void main1()
{
    if (setup1)
        setup1();
    while (true)
    {
        if (loop1)
            loop1();
    }
}

// extern SerialUSB serial;
/* file globals */

extern void fifoPioInit();
extern void networkInit();

void setup()
{
    gpio_init_mask(GPIO_CD_MASK | GPIO_CSW_MASK | GPIO_CSR_MASK | GPIO_A0_MASK);
    gpio_set_dir_in_masked(GPIO_CSW_MASK | GPIO_CSR_MASK | GPIO_A0_MASK);
    gpio_init(PIN_DIR);
    gpio_put(PIN_DIR, 1);
    gpio_set_dir(PIN_DIR, GPIO_OUT);
    gpio_set_drive_strength(PIN_DIR, GPIO_DRIVE_STRENGTH_12MA);
    gpio_pull_up(PIN_CD7);

    fifoPioInit();
    pio_sm_claim(FIFO_PIO, dmaRomSm);
    // serial.ignoreFlowControl();
#if USE_ETHERNET
    networkInit();
#endif
}

#define PICO_CLOCK_PLL 1260000000
#define PICO_CLOCK_PLL_DIV1 5
#define PICO_CLOCK_PLL_DIV2 1

int main()
{
    vreg_set_voltage(VREG_VOLTAGE_1_30);
    set_sys_clock_pll(PICO_CLOCK_PLL, PICO_CLOCK_PLL_DIV1, PICO_CLOCK_PLL_DIV2); // 252000
    //set_sys_clock_khz(320000, false);
    recursive_mutex_init(get_sd_mutex());
    stdio_init_all();
    // stdio_set_translate_crlf(&stdio_usb, false);

    multicore_launch_core1(main1);
    setup();
    while (true)
    {
        tight_loop_contents();
        loop();
    }
}
