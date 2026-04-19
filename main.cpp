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
extern "C" {
#include <common.h>
}
#include "fifo.pio.h"
#include "pico/sync.h"
#include "CircBuffer.h"

#include "tusb_config.h"
#include "tusb.h"
#include "scancode_rk.h"
#include "ws2812.h"
#include "init.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"

extern "C" {
#include "sd.h"
}


#define LEDBR 12

bool kb_enabled = true;
uint8_t kb_addr = 0;
uint8_t kb_inst = 0;

bool blinking = false;
//alarm_id_t repeater;

uint8_t prev_rpt[] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t prev_kb = 0;
uint8_t resend_kb = 0;
uint8_t resend_ms = 0;
uint8_t repeat = 0;


extern void setup();
extern void loop();

typedef struct
{
  tusb_desc_device_t desc_device;
  uint16_t manufacturer[32];
  uint16_t product[32];
  uint16_t serial[16];
  bool mounted;
} dev_info_t;
// CFG_TUH_DEVICE_MAX is defined by tusb_config header
dev_info_t dev_info[CFG_TUH_DEVICE_MAX] = {0};


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


extern void __not_in_flash_func(pio_irq_handler_write)();
extern void __not_in_flash_func(pio_irq_handler_rom_wr)();
extern void __not_in_flash_func(pio_irq_handler_rom_rd)();
extern CircBuffer<10 * 1024> bufIn;
extern CircBuffer<1024> bufOut;

int fifoReadProgOffset = 0;
int romReadProgramOffset = 0;
bool updateFifoReadAhead();

void setupFifoGpio()
{
    bufIn.clear();
    bufOut.clear();

    for (uint i = 0; i < 8; ++i)
    {
        pio_gpio_init(FIFO_PIO, PIN_CD7 + i);
        gpio_set_drive_strength(PIN_CD7 + i, GPIO_DRIVE_STRENGTH_12MA);
    }
    pio_gpio_init(FIFO_PIO, PIN_DIR);
}

void loadFifoReadProgram()
{
    setupFifoGpio();
    pio_sm_claim(FIFO_PIO, fifoReadSm);
    pio_sm_clear_fifos(FIFO_PIO, fifoReadSm);
    fifoReadProgOffset = pio_add_program(FIFO_PIO, &fifoRead_program);
    if (fifoReadProgOffset < 0)
        panic("Failed add fifoReadProgram");

    //pio_sm_set_pins_with_mask(FIFO_PIO, fifoReadSm, DIR_MASK, DIR_MASK);
    //pio_sm_set_pindirs_with_mask(FIFO_PIO, fifoReadSm, DIR_MASK, DIR_MASK);
     //hw_set_bits(&FIFO_PIO->input_sync_bypass, (uint32_t)-1/* (1u << PIN_A0_28) | (1u << PIN_CSR) */);
    /* fifoReadProg */
    pio_sm_config readFifoConfig = fifoRead_program_get_default_config(fifoReadProgOffset);
    //sm_config_set_in_pins(&readFifoConfig, PIN_CSR);
    sm_config_set_jmp_pin(&readFifoConfig, PIN_A0_28);
    sm_config_set_mov_status(&readFifoConfig, STATUS_TX_LESSTHAN, 1);
    sm_config_set_fifo_join(&readFifoConfig, PIO_FIFO_JOIN_TX);
    sm_config_set_sideset_pin_base(&readFifoConfig, PIN_DIR);
    sm_config_set_out_pins(&readFifoConfig, PIN_CD7, 8);
    sm_config_set_in_shift(&readFifoConfig, true, false, 32);  // R shift
    sm_config_set_out_shift(&readFifoConfig, true, false, 32); // R shift
    sm_config_set_clkdiv(&readFifoConfig, 1.0f);
    //
    pio_sm_init(FIFO_PIO, fifoReadSm, fifoReadProgOffset, &readFifoConfig);
    // pio_set_irq0_source_enabled(FIFO_PIO, pis_sm0_rx_fifo_not_empty, true);
    // irq_set_exclusive_handler(PIO0_IRQ_0, pio_irq_handler_read);
    //irq_set_enabled(PIO0_IRQ_0, false);
    pio_sm_set_consecutive_pindirs(FIFO_PIO, fifoReadSm, PIN_CD7, 8, false);
    //pio_set_irq0_source_enabled(FIFO_PIO, pis_sm2_rx_fifo_not_empty, false);
    pio_sm_set_enabled(FIFO_PIO, fifoReadSm, true);
   //updateFifoReadAhead();
}



void loadFifoWriteProgram()
{
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
    const uint irq = PIO1_IRQ_1;
    irq_set_exclusive_handler(irq, pio_irq_handler_write);
    irq_set_enabled(irq, true);
}
/*
 * Set up PIOs for pico <-> CPU interface
 */
void fifoPioInit()
{
    setupFifoGpio();
#ifdef USE_SERIAL_DEBUG
    loadFifoReadProgram();
#endif
    loadFifoWriteProgram();
}

/*
 * Set up PIOs for pico <-> CPU interface
 * FIFO pio for read-like programs
 * DMA pio for write-like due to the same used pins
 */
extern void __not_in_flash_func(updateTX)();
void loadKbdProgram()
{
    pio_sm_claim(FIFO_PIO, fifoRomRdSm);
    PIO pio = FIFO_PIO;
    uint irq = PIO0_IRQ_0;
    romReadProgramOffset = pio_add_program(pio, &romRead_program);
    if (romReadProgramOffset < 0)
        panic("Failed add fifoReadProgram");
    pio_sm_clear_fifos(pio, fifoRomRdSm);

    pio_sm_config romConfig = romRead_program_get_default_config(romReadProgramOffset);
    sm_config_set_in_pins(&romConfig, PIN_A0);
    sm_config_set_jmp_pin(&romConfig, PIN_nWR);
    sm_config_set_sideset_pin_base(&romConfig, PIN_DIR);
    sm_config_set_out_pins(&romConfig, PIN_CD7, 8);
#define SH_LEFT false
#define SH_RIGHT true
    sm_config_set_in_shift(&romConfig, SH_LEFT, false, 32);   // L shift
    sm_config_set_out_shift(&romConfig, SH_RIGHT, false, 32); // R shift
    sm_config_set_clkdiv(&romConfig, 1.0f);

    pio_sm_init(pio, fifoRomRdSm, romReadProgramOffset, &romConfig);
    pio_set_irq0_source_enabled(pio, pis_sm2_rx_fifo_not_empty, true);
    irq_set_exclusive_handler(irq, pio_irq_handler_rom_rd);
    irq_set_enabled(irq, true);
    pio_sm_set_enabled(pio, fifoRomRdSm, true /* false */);
    //enable_interrupts();
    updateTX();
}

void dmaPioInit()
{
#if !USE_DMA || KBD_EMU
    // DMA Read  = SD  -> Mem
    // DMA Write = Mem -> SD
    loadKbdProgram();
#endif
#if USE_DMA
    uint dmaWriteProgOffset = pio_add_program(DMA_PIO, &dmaWrite_program);
     if (dmaWriteProgOffset < 0)
        panic("Failed add dmaWriteProgram");
   
    pio_sm_config writeConfig = dmaWrite_program_get_default_config(dmaWriteProgOffset);
    //sm_config_set_jmp_pin(&writeConfig, PIN_nDACK);
    sm_config_set_fifo_join(&writeConfig, PIO_FIFO_JOIN_RX);
    sm_config_set_in_pins(&writeConfig, PIN_CD7);
    sm_config_set_in_shift(&writeConfig, SH_LEFT, true/*Autopush*/, 16); // L shift, autopush @ 16 bits
    sm_config_set_clkdiv(&writeConfig, 1.0f);

    pio_sm_init(DMA_PIO, dmaWriteSm, dmaWriteProgOffset, &writeConfig);
    pio_sm_set_enabled(DMA_PIO, dmaWriteSm, true/* false */);
    //
    int dmaReadProgOffset = pio_add_program(FIFO_PIO, &dmaRead_program);
    if (dmaReadProgOffset<0)
        panic("Failed add dmaReadProgram");

    pio_gpio_init(FIFO_PIO, PIN_DIR);
    //pio_gpio_init(FIFO_PIO, PIN_nWAIT);
    uint32_t mask = DIR_MASK;// | nWAIT_MASK;
    pio_sm_set_pins_with_mask(FIFO_PIO, dmaReadSm, mask, mask);
    pio_sm_set_pindirs_with_mask(FIFO_PIO, dmaReadSm, mask, mask);

    pio_sm_config readDmaConfig = dmaRead_program_get_default_config(dmaReadProgOffset);
    //sm_config_set_in_pins(&readDmaConfig, PIN_CSR);
    //sm_config_set_jmp_pin(&readDmaConfig, PIN_nDACK);
    sm_config_set_fifo_join(&readDmaConfig, PIO_FIFO_JOIN_TX);
    sm_config_set_sideset_pin_base(&readDmaConfig, PIN_DIR);
    sm_config_set_out_pins(&readDmaConfig, PIN_CD7, 8);
    //sm_config_set_set_pins(&readDmaConfig, PIN_nWAIT, 1);
    sm_config_set_in_shift(&readDmaConfig, true, false, 32); // R shift
    sm_config_set_out_shift(&readDmaConfig, true, false, 32); // R shift
    sm_config_set_clkdiv(&readDmaConfig, 1.0f);

    pio_sm_init(FIFO_PIO, dmaReadSm, dmaReadProgOffset, &readDmaConfig);
    pio_sm_set_enabled(FIFO_PIO, dmaReadSm, true);
#endif

    PIO pioRwr = DMA_PIO;
    const uint smRwr = dmaRomWrSm;
    pio_sm_claim(pioRwr, smRwr);
    uint irqRwr = PIO1_IRQ_0;
    int romWriteProgramOffset = pio_add_program(pioRwr, &romWrite_program);
    if (romWriteProgramOffset < 0)
        panic("Failed add romWrite_program");
    pio_sm_config romConfigWr = romWrite_program_get_default_config(romWriteProgramOffset);
    sm_config_set_in_pins(&romConfigWr, PIN_A0);
    sm_config_set_in_shift(&romConfigWr, SH_LEFT, false, 32);   // L shift
    sm_config_set_out_shift(&romConfigWr, SH_RIGHT, false, 32); // R shift
    sm_config_set_clkdiv(&romConfigWr, 1.0f);

    pio_sm_init(pioRwr, smRwr, romWriteProgramOffset, &romConfigWr);
    pio_set_irq0_source_enabled(pioRwr, pis_sm3_rx_fifo_not_empty, true);
    irq_set_exclusive_handler(irqRwr, pio_irq_handler_rom_wr);
    irq_set_enabled(irqRwr, true);
    pio_sm_set_enabled(pioRwr, smRwr, true /* false */);
}


// Picoboard has a button attached to the flash CS pin, which the bootrom
// checks, and jumps straight to the USB bootcode if the button is pressed
// (pulling flash CS low). We can check this pin in by jumping to some code in
// SRAM (so that the XIP interface is not required), floating the flash CS
// pin, and observing whether it is pulled low.
//
// This doesn't work if others are trying to access flash at the same time,
// e.g. XIP streamer, or the other core.

bool __no_inline_not_in_flash_func(get_usr_button)() {
#if 0
    const uint CS_PIN_INDEX = 1;

    // Must disable interrupts, as interrupt handlers may be in flash, and we
    // are about to temporarily disable flash access!
    uint32_t flags = save_and_disable_interrupts();

    // Set chip select to Hi-Z
    hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,
                    GPIO_OVERRIDE_LOW << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

    // Note we can't call into any sleep functions in flash right now
    for (volatile int i = 0; i < 1000; ++i);

    // The HI GPIO registers in SIO can observe and control the 6 QSPI pins.
    // Note the button pulls the pin *low* when pressed.
#if PICO_RP2040
    #define CS_BIT (1u << 1)
#else
    #define CS_BIT SIO_GPIO_HI_IN_QSPI_CSN_BITS
#endif
    bool button_state = !(sio_hw->gpio_hi_in & CS_BIT);

    // Need to restore the state of chip select, else we are going to have a
    // bad time when we return to code in flash!
    hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,
                    GPIO_OVERRIDE_NORMAL << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

    restore_interrupts(flags);
#else
#if 0
    gpio_set_dir(PIN_LED, GPIO_IN);
    //sleep_ms(10);
    for (volatile int i = 0; i < 1000; ++i);
    bool button_state = gpio_get(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
#else
    bool button_state = !gpio_get(PIN_USR_KEY);
#endif
#endif
    return button_state;
}

#ifdef KBD_EMU
bool bKbdEmu = true;
#else
bool bKbdEmu = false;
#endif


void switchConfig()
{
    bKbdEmu = !bKbdEmu;
    if (bKbdEmu)
    {
        //pio_remove_program_and_unclaim_sm( &rom2_program, FIFO_PIO, dmaRomSm, romProgramOffset);
        pio_sm_set_enabled(FIFO_PIO, fifoReadSm, false);
        pio_remove_program_and_unclaim_sm( &fifoRead_program, FIFO_PIO, fifoReadSm, fifoReadProgOffset);
        loadKbdProgram();

    }
    else
    {
        pio_sm_set_enabled(FIFO_PIO, fifoRomRdSm, false);
        pio_remove_program_and_unclaim_sm( &romRead_program, FIFO_PIO, fifoRomRdSm, romReadProgramOffset);
        loadFifoReadProgram();
    }
    LedUpdate();
}

extern void networkInit();

void setup()
{
    gpio_init_mask(GPIO_CD_MASK | GPIO_CSW_MASK | GPIO_CSR_MASK | GPIO_A0_MASK | GPIO_A1_MASK);
    gpio_set_dir_in_masked(GPIO_CSW_MASK | GPIO_CSR_MASK | GPIO_A0_MASK | GPIO_A1_MASK);
    gpio_init(PIN_DIR);
    gpio_put(PIN_DIR, 1);
    gpio_set_dir(PIN_DIR, GPIO_OUT);
    gpio_set_drive_strength(PIN_DIR, GPIO_DRIVE_STRENGTH_12MA);
    gpio_pull_up(PIN_CD7);

    gpio_init(PIN_USR_KEY);
    gpio_set_dir(PIN_USR_KEY, false);
    gpio_set_pulls(PIN_USR_KEY, true, false);

    dmaPioInit();
    fifoPioInit();
    multicore_fifo_pop_blocking();
#if USE_ETHERNET
    networkInit();
#endif
    multicore_fifo_push_blocking(0);
}

void setup1()
{
    spi_init(_SPI, BAUD);
    spi_set_format(_SPI, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(PIN_SPI_RX, GPIO_FUNC_SPI);
    // gpio_set_function(PIN_SPI_CSn, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_TX, GPIO_FUNC_SPI);
    sd_init();
    multicore_fifo_push_blocking(1);

    gpio_init_mask(A0_MASK | A1_MASK | /* nCS2_MASK | GPIO_CD_MASK |  */ nWR_MASK | nRD_MASK | nIOR_MASK | nIOR_MASK);

    gpio_init(PIN_SPI_CSn);
    gpio_put(PIN_SPI_CSn, 1);
    gpio_set_dir(PIN_SPI_CSn, GPIO_OUT);

    gpio_init(PIN_DRQ);
    gpio_put(PIN_DRQ, 0);
    gpio_set_dir(PIN_DRQ, GPIO_OUT);

    // gpio_init(PIN_nWAIT);
    // gpio_put(PIN_nWAIT, 1);
    // gpio_set_dir(PIN_nWAIT, GPIO_OUT);
    //gpio_set_drive_strength(PIN_nWAIT, GPIO_DRIVE_STRENGTH_12MA);
    //gpio_pull_down(PIN_nWAIT);

    //gpio_init(PIN_nDACK);
    //gpio_set_dir(PIN_nDACK, GPIO_IN);

    gpio_init(PIN_nIOR);
    gpio_set_dir(PIN_nIOR, GPIO_IN);

    gpio_init(PIN_nIOW);
    gpio_set_dir(PIN_nIOW, GPIO_IN);

    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    //gpio_put(PIN_LED, 0);
    LedOff();
    // SD cards' DO MUST be pulled up.
    gpio_pull_up(PIN_SPI_RX);
}


void main1()
{
    // if (setup1)
    setup1();
    while (true)
    {
        main_sd();
    }
}

// extern SerialUSB serial;
/* file globals */

extern void fifoPioInit();
void setupFifoGpio();
extern void networkInit();


#define PICO_CLOCK_PLL 1260000000
#define PICO_CLOCK_PLL_DIV1 5
#define PICO_CLOCK_PLL_DIV2 1

///////////////////////////////////////////////////////////////////////////////////////////////
//------------------------------------------------------------------------------
uint8_t type_hid = 0;
//------------------------------------------------------------------------------
// void mouse(uint8_t const *report, uint16_t len )
void mouse(hid_mouse_report_t const *report, uint16_t len)
{
  // debug_print("B=%02X X=%02d Y=%02d #1F=%02X        %02X\r\n", mouse_b, mouse_x, mouse_y, joy_k ,type_hid );
}
//------------------------------------------------------------------

/*
 * Your mouse's polling rate.
 * If you don't know what yours is, follow this link:
 * https://wiki.archlinux.org/index.php/Mouse_polling_rate
 */
#define POLLING_RATE 125

/*
 * This should be your desired acceleration. It needs to end with an f.
 * For example, setting this to "0.1f" should be equal to
 * cl_mouseaccel 0.1 in Quake.
 */
#define ACCELERATION 0.2f

#define SENSITIVITY 1.0f
#define SENS_CAP 0.0f
#define OFFSET 0.0f
#define PRE_SCALE_X 1.0f
#define PRE_SCALE_Y 1.0f
#define POST_SCALE_X 1.0f
#define POST_SCALE_Y 1.0f
#define SPEED_CAP 0.0f



static inline int Leet_round(float x)
{
	if (x >= 0) {
		return (int)(x + 0.5f);
	} else {
		return (int)(x - 0.5f);
	}
}

// What do we have here? Code from Quake 3, which is also GPL.
// https://en.wikipedia.org/wiki/Fast_inverse_square_root
// Copyright (C) 1999-2005 Id Software, Inc.
static inline float Q_sqrt(float number)
{
	long i;
	float x2, y;
	const float threehalfs = 1.5F;

	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;                       // evil floating point bit level hacking
	i  = 0x5f3759df - ( i >> 1 );               // what the fuck?
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
//	y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed

	return 1 / y;
}


static void __noinline process_mouse_report(hid_mouse_report_t const * report)
{
	static hid_mouse_report_t prev_report = { 0 };

	// Mouse position.
	//printf("Mouse: (%d %d %d)", report->x, report->y, report->wheel);
	// acceleration happens here
	float delta_x = report->x * PRE_SCALE_X;
	float delta_y = report->y * PRE_SCALE_Y;

#if 1
	float ms = 1000.0f / POLLING_RATE;
	float accel_sens = SENSITIVITY;
	float rate = Q_sqrt(delta_x * delta_x + delta_y * delta_y);
	static float carry_x = 0.0f;
	static float carry_y = 0.0f;

	if (SPEED_CAP != 0) {
		if (rate >= SPEED_CAP) {
			delta_x *= SPEED_CAP / rate;
			delta_y *= SPEED_CAP / rate;
		}
	}
	rate /= ms;
	rate -= OFFSET;
	if (rate > 0) {
		rate *= ACCELERATION;
		accel_sens += rate;
	}
	if (SENS_CAP > 0 && accel_sens >= SENS_CAP) {
		accel_sens = SENS_CAP;
	}
	accel_sens /= SENSITIVITY;
	delta_x *= accel_sens;
	delta_y *= accel_sens;
	delta_x *= POST_SCALE_X;
	delta_y *= POST_SCALE_Y;
	delta_x += carry_x;
	delta_y += carry_y;
	carry_x = delta_x - Leet_round(delta_x);
	carry_y = delta_y - Leet_round(delta_y);
#endif
	// Button state.
    mouseXAbs += Leet_round(delta_x);
    mouseYAbs += Leet_round(delta_y);
    mouseButtons = report->buttons;
    updateTX();
	uint8_t button_changed_mask = report->buttons ^ prev_report.buttons;
	if(button_changed_mask & report->buttons) {
		// printf(" %c%c%c",
		//        report->buttons & MOUSE_BUTTON_LEFT   ? 'L' : '-',
		//        report->buttons & MOUSE_BUTTON_MIDDLE ? 'M' : '-',
		//        report->buttons & MOUSE_BUTTON_RIGHT  ? 'R' : '-');
	}
    prev_report = *report;
	//printf("\n");
}
//-----------------------------------------------------------------------------
// Вызывается при получении отчета от устройства через конечную точку прерывания
// Примечание: если есть идентификатор отчета (составной), то это 1-й байт отчета
void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const *report, uint16_t len)
{
  switch (tuh_hid_interface_protocol(dev_addr, instance))
  {

  case HID_ITF_PROTOCOL_KEYBOARD:
    kb_addr = dev_addr;
    kb_inst = instance;
    rk_keyboard((const hid_keyboard_report_t *)report, len);
    break;

  case HID_ITF_PROTOCOL_MOUSE: // mouse(report, len);break;
  {
      process_mouse_report((hid_mouse_report_t const *)report);
      break;
  }
  default:
      break; // gamepad (report,len);break;
             // continue to request to receive report
  }
  if (!tuh_hid_receive_report(dev_addr, instance))
      panic("Error: cannot request to receive report\r\n");
}




///////////////////////////////////////////////////////////////////////////////////////////
// Invoked when device is mounted (configured)
/*  void tuh_mount_cb (uint8_t daddr)
{
   if(DEBUG) printf("tuh_mount_cb = %d\r\n", daddr);


} */

///////////////////////////////////////////////////////////////////////////////////////////
// Вызывается при подключении устройства с интерфейсом hid
// Дескриптор отчета также доступен для использования. tuh_hid_parse_report_descriptor()
// может использоваться для анализа общего /достаточно простого дескриптора.
// Примечание: если длина дескриптора отчета > CFG_TUH_ENUMERATION_BUFSIZE, он будет пропущен
// следовательно, report_desc = NULL, desc_len = 0
void tuh_hid_mount_cb(uint8_t daddr, uint8_t instance, uint8_t const *desc_report, uint16_t desc_len)
{
  //debug_print("HID device address = %d, instance = %d is mounted\n", daddr, instance);

  dev_info_t *dev = &dev_info[daddr - 1];
  dev->mounted = true;

  // Get Device Descriptor
  // tuh_descriptor_get_device(daddr, &dev->desc_device, 18, print_device_descriptor, 0);

  switch (tuh_hid_interface_protocol(daddr, instance))
  {
  case HID_ITF_PROTOCOL_KEYBOARD:
    //debug_print("HID Interface Protocol = Keyboard\n");

    kb_addr = daddr;
    kb_inst = instance;

    tuh_hid_receive_report(daddr, instance);
    ws2812_set_rgb(0, LEDBR, LEDBR);
    break;

  case HID_ITF_PROTOCOL_MOUSE:
    //debug_print("HID Interface Protocol = Mouse\n");

    tuh_hid_receive_report(daddr, instance);
    break;
  }
}
// Вызывается, когда устройство с интерфейсом hid не подключено
void tuh_hid_umount_cb(uint8_t daddr, uint8_t instance)
{
  //debug_print("HID device address = %d, instance = %d is unmounted\r\n", daddr, instance);
  dev_info_t *dev = &dev_info[daddr - 1];
  dev->mounted = false;

  // print device summary
  //print_lsusb();

  ws2812_set_rgb(LEDBR, 0, 0);
}

uint32_t lastTimestamp = to_ms_since_boot(get_absolute_time());

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
#if 1//def KBD_EMU
    tusb_init(); // инициализация USB OTG
    kb_update_leds();
#endif
    while (true)
    {
        //tight_loop_contents();
        loop();
//#ifdef KBD_EMU
        tuh_task();
//#endif
        uint32_t timestamp = to_ms_since_boot(get_absolute_time());
        if ((timestamp - lastTimestamp) > 50)
        {
            bool bButton = false;
            bButton = get_usr_button();
            static bool bLastButton = false;
            if (bButton && !bLastButton)
            {
                MTX_ENTER();
                switchConfig();
                MTX_EXIT();
            }
            bLastButton = bButton;
            lastTimestamp = timestamp;
        }
    }
}
