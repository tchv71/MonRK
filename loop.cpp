#include "hardware/pio.h"
#include "Serial.h"
#include "gpios.h"
#include "fifo.pio.h"
#include "common.h"
#include "hardware/sync.h"

extern SerialUSB serial;

const uint8_t RXEMPTY = 1; // MASK FOR RX BUFFER EMPTY
const uint8_t TXFULL = 2;  // MASK FOR TX BUFFER FULL

static volatile uint8_t nextValue = 0;         /* pico read-ahead value */
static volatile uint8_t currentStatus = RXEMPTY; /* current status register value */


/*
 * update the value send to the read PIO
 */
inline static void updateFifoReadAhead()
{
    uint32_t readAhead = nextValue;
    readAhead |= currentStatus << 8;
    readAhead |= 0xFF << 16; // pin direction
   
    pio_sm_put(FIFO_PIO, fifoReadSm, readAhead);
}

static uint8_t streamInBuf[10*1024];
static uint8_t *pStreamInBufEnd;
static uint8_t *pStreamInBufPtr;

static uint8_t streamOutBuf[1024];
static uint8_t *pStreamOutBufPtr;

static volatile bool bFlushOutBuffer = false;
static volatile uint8_t outLength = 0xff;
extern "C" const uint fifoWrite2Sm;

/*
 * handle interrupts from the pico<->CPU interface
 */
void __not_in_flash_func(pio_irq_handler_write)()
{
    uint32_t writeVal =  DMA_PIO->rxf[fifoWrite2Sm];

    if ((writeVal & (GPIO_A0_MASK >> GPIO_CD7)) == 0) // write val
    {
        uint8_t c = writeVal & 0xff;
        *pStreamOutBufPtr++ = c;
        uint8_t pos = pStreamOutBufPtr - streamOutBuf;
        if (pos == 2)
            outLength = c + 3;
        if (pos == outLength)
        {
            bFlushOutBuffer = true;
            //currentStatus |= RXEMPTY;
        }
    }
    if (pStreamOutBufPtr == streamOutBuf + sizeof(streamOutBuf))
        currentStatus |= TXFULL;
    updateFifoReadAhead();
}

void __not_in_flash_func(pio_irq_handler_read)()
{
    uint32_t readVal = pio_sm_get_blocking(FIFO_PIO, fifoReadSm);//FIFO_PIO->rxf[fifoReadSm];

    if (!(readVal &  0x80000000/* 0x04 */)) // read data
    {
        if (pStreamInBufPtr != pStreamInBufEnd)
        {
            if ((currentStatus & RXEMPTY) == 0)
            {
                nextValue = *pStreamInBufPtr++;
            }
        }
        else
        {
            currentStatus |= RXEMPTY;
            pStreamInBufPtr = pStreamInBufEnd = streamInBuf;
            nextValue = 0;
        }
    }
    updateFifoReadAhead();
}

/*
 * Set up PIOs for pico <-> CPU interface
 */
void fifoPioInit()
{
    pStreamInBufPtr = pStreamInBufEnd = streamInBuf;
    pStreamOutBufPtr = streamOutBuf;


    int fifoReadProgOffset = pio_add_program(FIFO_PIO, &fifoRead_program);
    if (fifoReadProgOffset<0)
        panic("Failed add fifoReadProgram");

    for (uint i = 0; i < 8; ++i)
    {
        pio_gpio_init(FIFO_PIO, GPIO_CD7 + i);
    }
    pio_gpio_init(FIFO_PIO, DIR);
    pio_sm_set_pins_with_mask(FIFO_PIO, fifoReadSm, DIR_MASK, DIR_MASK);
    pio_sm_set_pindirs_with_mask(FIFO_PIO, fifoReadSm, DIR_MASK, DIR_MASK);
    //pio_sm_set_consecutive_pindirs(FIFO_PIO, fifoReadSm, DIR, 1, true);

    pio_sm_config readFifoConfig = fifoRead_program_get_default_config(fifoReadProgOffset);
    sm_config_set_in_pins(&readFifoConfig, GPIO_CSR);
    sm_config_set_jmp_pin(&readFifoConfig, GPIO_A0);
    sm_config_set_sideset_pin_base(&readFifoConfig, DIR);
    sm_config_set_out_pins(&readFifoConfig, GPIO_CD7, 8);
    sm_config_set_in_shift(&readFifoConfig, true, false, 32); // R shift
    sm_config_set_out_shift(&readFifoConfig, true, false, 32); // R shift
    sm_config_set_clkdiv(&readFifoConfig, 1.0f);

    pio_sm_init(FIFO_PIO, fifoReadSm, fifoReadProgOffset, &readFifoConfig);
    pio_sm_set_enabled(FIFO_PIO, fifoReadSm, true);
    pio_set_irq0_source_enabled(FIFO_PIO, pis_sm0_rx_fifo_not_empty, true);
    irq_set_exclusive_handler(PIO0_IRQ_0, pio_irq_handler_read);
    irq_set_enabled(PIO0_IRQ_0, true);

    irq_set_exclusive_handler(PIO1_IRQ_1, pio_irq_handler_write);
    irq_set_enabled(PIO1_IRQ_1, true);
    int fifoWriteProgOffset = pio_add_program(DMA_PIO, &fifoWrite_program);
    if (fifoWriteProgOffset<0)
        panic("Failed add fifoWriteProgram");

    pio_sm_config writeConfig2 = fifoWrite_program_get_default_config(fifoWriteProgOffset);
    sm_config_set_in_pins(&writeConfig2, GPIO_CD7);
    sm_config_set_in_shift(&writeConfig2, false, true, 16); // L shift, autopush @ 16 bits
    sm_config_set_clkdiv(&writeConfig2, 1.0f);

    pio_sm_init(DMA_PIO, fifoWrite2Sm, fifoWriteProgOffset, &writeConfig2);
    pio_sm_set_enabled(DMA_PIO, fifoWrite2Sm, true);
    pio_set_irq1_source_enabled(DMA_PIO, pis_sm1_rx_fifo_not_empty, true);

    updateFifoReadAhead();
}

void loop()
{
    if (serial.available() && ((currentStatus & TXFULL) == 0) && pStreamInBufPtr==pStreamInBufEnd)
    {
        if (pStreamInBufEnd == pStreamInBufPtr)
            pStreamInBufEnd = pStreamInBufPtr = streamInBuf;
        while (serial.available() && pStreamInBufEnd != streamInBuf + sizeof(streamInBuf))
        {
            *pStreamInBufEnd++ = serial.read();
        }
        uint32_t ints = save_and_disable_interrupts();
        // if (currentStatus & RXEMPTY)
        //     pio_sm_clear_fifos(FIFO_PIO, fifoReadSm);
        nextValue = *pStreamInBufPtr++;
        currentStatus &= ~RXEMPTY;
        outLength = 0xFF;
        updateFifoReadAhead();
        restore_interrupts_from_disabled(ints);
    }
    if ((pStreamOutBufPtr == streamOutBuf + sizeof(streamOutBuf) || (bFlushOutBuffer && pStreamOutBufPtr != streamOutBuf)) && serial.availableForWrite())
    {
        currentStatus |= TXFULL/*  | RXEMPTY */;
        serial.write(streamOutBuf, pStreamOutBufPtr - streamOutBuf);
        serial.flush();
        pStreamOutBufPtr = streamOutBuf;
        currentStatus &= ~TXFULL;
        bFlushOutBuffer = false;
        updateFifoReadAhead();
    }
}
