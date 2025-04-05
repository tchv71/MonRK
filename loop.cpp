#include "hardware/pio.h"
#include "Serial.h"
#include "gpios.h"
#include "fifo.pio.h"

extern SerialUSB serial;

static volatile uint8_t nextValue = 0;         /* pico read-ahead value */
static volatile uint8_t currentStatus = RXEMPTY; /* current status register value */

extern const uint fifoWriteSm = 0;
extern const uint fifoReadSm = 1;

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
/*
 * handle interrupts from the pico<->CPU interface
 */
void __not_in_flash_func(pio_irq_handler_write)()
{
    uint32_t writeVal =  FIFO_PIO->rxf[fifoWriteSm];

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
            currentStatus |= RXEMPTY;
        }
    }
    if (pStreamOutBufPtr == streamOutBuf + sizeof(streamOutBuf))
        currentStatus |= TXFULL;
    updateFifoReadAhead();
}
bool bTempBlock = false;

void __not_in_flash_func(pio_irq_handler_read)()
{
    uint32_t readVal = pio_sm_get_blocking(FIFO_PIO, fifoReadSm);//FIFO_PIO->rxf[fifoReadSm];

    if ((readVal & 0x04) == 0) // read data
    {
        if (pStreamInBufPtr != pStreamInBufEnd)
        {
            if ((currentStatus & RXEMPTY) == 0)
            {
                nextValue = *pStreamInBufPtr++;
                bTempBlock = true;
                currentStatus |= RXEMPTY;
            }
        }
        else
        {
            currentStatus |= RXEMPTY;
            pStreamInBufPtr = pStreamInBufEnd = streamInBuf;
            nextValue = 0;
        }
    }
    else if ((currentStatus & RXEMPTY) && bTempBlock)
    {
        currentStatus &= ~RXEMPTY;
        bTempBlock = false;
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

    irq_set_exclusive_handler(FIFO_IRQ, pio_irq_handler_write);
    irq_set_enabled(FIFO_IRQ, true);

    uint fifoWriteProgram = pio_add_program(FIFO_PIO, &fifoWrite_program);

    pio_sm_config writeConfig = fifoWrite_program_get_default_config(fifoWriteProgram);
    sm_config_set_in_pins(&writeConfig, GPIO_CD7);
    sm_config_set_in_shift(&writeConfig, false, true, 16); // L shift, autopush @ 16 bits
    sm_config_set_clkdiv(&writeConfig, 1.0f);

    pio_sm_init(FIFO_PIO, fifoWriteSm, fifoWriteProgram, &writeConfig);
    pio_sm_set_enabled(FIFO_PIO, fifoWriteSm, true);
    pio_set_irq0_source_enabled(FIFO_PIO, pis_sm0_rx_fifo_not_empty, true);

    irq_set_exclusive_handler(PIO0_IRQ_1, pio_irq_handler_read);
    irq_set_enabled(PIO0_IRQ_1, true);

    uint fifoReadProgram = pio_add_program(FIFO_PIO, &fifoRead_program);

    for (uint i = 0; i < 8; ++i)
    {
        pio_gpio_init(FIFO_PIO, GPIO_CD7 + i);
    }

    pio_sm_config readConfig = fifoRead_program_get_default_config(fifoReadProgram);
    sm_config_set_in_pins(&readConfig, GPIO_CSR);
    sm_config_set_jmp_pin(&readConfig, GPIO_A0);
    sm_config_set_out_pins(&readConfig, GPIO_CD7, 8);
    sm_config_set_in_shift(&readConfig, false, false, 32); // L shift
    sm_config_set_out_shift(&readConfig, true, false, 32); // R shift
    sm_config_set_clkdiv(&readConfig, 4.0f);

    pio_sm_init(FIFO_PIO, fifoReadSm, fifoReadProgram, &readConfig);
    pio_sm_set_enabled(FIFO_PIO, fifoReadSm, true);
    pio_set_irq1_source_enabled(FIFO_PIO, pis_sm1_rx_fifo_not_empty, true);

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
        nextValue = *pStreamInBufPtr++;
        currentStatus &= ~RXEMPTY;
        outLength = 0xFF;
        updateFifoReadAhead();
    }
    if ((pStreamOutBufPtr == streamOutBuf + sizeof(streamOutBuf) || (bFlushOutBuffer && pStreamOutBufPtr != streamOutBuf)) && serial.availableForWrite())
    {
        currentStatus = TXFULL | RXEMPTY;
        serial.write(streamOutBuf, pStreamOutBufPtr - streamOutBuf);
        serial.flush();
        pStreamOutBufPtr = streamOutBuf;
        currentStatus &= ~TXFULL;
        bFlushOutBuffer = false;
        updateFifoReadAhead();
    }
}
