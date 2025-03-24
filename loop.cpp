#include "hardware/pio.h"
#include "Serial.h"
#include "gpios.h"
#include "fifo.pio.h"

extern SerialUSB serial;

static volatile uint8_t nextValue = 0;         /* pico read-ahead value */
static volatile uint8_t currentStatusRead = 0; /* current status register value */
static volatile uint8_t currentStatusWrite = 0;

extern const uint fifoWriteSm = 0;
extern const uint fifoReadSm = 1;

/*
 * update the value send to the read PIO
 */
inline static void updateFifoReadAhead()
{
    // nextValue = 0;
    uint8_t currentStatus = currentStatusRead | currentStatusWrite;
    uint32_t readAhead = 0xff; // pin direction
    readAhead |= nextValue << 8;
    readAhead |= currentStatus << 16;
    pio_sm_put(FIFO_PIO, fifoReadSm, readAhead);
}

static uint8_t streamInBuf[1024];
static uint8_t *pStreamInBufEnd;
static uint8_t *pStreamInBufPtr;

static uint8_t streamOutBuf[1024];
static uint8_t *pStreamOutBufPtr;

static volatile bool bFlushOutBuffer = false;
static volatile uint8_t outLength = 0xff;
/*
 * handle interrupts from the pico<->CPU interface
 */
void __not_in_flash_func(pio_irq_handler)()
{
    if ((FIFO_PIO->fstat & (1u << (PIO_FSTAT_RXEMPTY_LSB + fifoWriteSm))) == 0) // write?
    {
        uint32_t writeVal = FIFO_PIO->rxf[fifoWriteSm];

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
                currentStatusRead = RXEMPTY;
            }
        }
        if (pStreamOutBufPtr == streamOutBuf + sizeof(streamOutBuf))
            currentStatusWrite = TXFULL;
        updateFifoReadAhead();
    }
    else if ((FIFO_PIO->fstat & (1u << (PIO_FSTAT_RXEMPTY_LSB + fifoReadSm))) == 0) // read?
    {
        uint32_t readVal = FIFO_PIO->rxf[fifoReadSm];

        if ((readVal & 0x04) == 0) // read data
        {
            // if (pStreamOutBufPtr != streamOutBuf && !bFlushOutBuffer)
            // {
            //     bFlushOutBuffer = true;
            //     currentStatusRead = RXEMPTY;
            // }
            // currentStatusRead = 0;
            if (pStreamInBufPtr != pStreamInBufEnd)
                nextValue = *pStreamInBufPtr++;
            else
            {
                currentStatusRead = RXEMPTY;
                pStreamInBufPtr = pStreamInBufEnd = streamInBuf;
            }
        }
        // else if (pStreamOutBufPtr != streamOutBuf)// read status
        //     bFlushOutBuffer = true;
        updateFifoReadAhead();
    }
}

/*
 * Set up PIOs for pico <-> CPU interface
 */
void fifoPioInit()
{
    pStreamInBufPtr = pStreamInBufEnd = streamInBuf;
    pStreamOutBufPtr = streamOutBuf;

    irq_set_exclusive_handler(FIFO_IRQ, pio_irq_handler);
    irq_set_enabled(FIFO_IRQ, true);

    uint fifoWriteProgram = pio_add_program(FIFO_PIO, &fifoWrite_program);

    pio_sm_config writeConfig = fifoWrite_program_get_default_config(fifoWriteProgram);
    sm_config_set_in_pins(&writeConfig, GPIO_CD7);
    sm_config_set_in_shift(&writeConfig, false, true, 16); // L shift, autopush @ 16 bits
    sm_config_set_clkdiv(&writeConfig, 1.0f);

    pio_sm_init(FIFO_PIO, fifoWriteSm, fifoWriteProgram, &writeConfig);
    pio_sm_set_enabled(FIFO_PIO, fifoWriteSm, true);
    pio_set_irq0_source_enabled(FIFO_PIO, pis_sm0_rx_fifo_not_empty, true);

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
    pio_set_irq0_source_enabled(FIFO_PIO, pis_sm1_rx_fifo_not_empty, true);

    pio_sm_put(FIFO_PIO, fifoReadSm, 0x000100ff);
}

void loop()
{
    if (pStreamInBufEnd == pStreamInBufPtr && serial.available() && currentStatusWrite == 0)
    {
        pStreamInBufEnd = pStreamInBufPtr = streamInBuf;
        while (serial.available())
        {
            *pStreamInBufEnd++ = serial.read();
        }
        nextValue = *pStreamInBufPtr++;
        currentStatusRead = 0;
        outLength = 0xFF;
        updateFifoReadAhead();
    }
    if ((pStreamOutBufPtr == streamOutBuf + sizeof(streamOutBuf) || (bFlushOutBuffer && pStreamOutBufPtr != streamOutBuf)) && serial.availableForWrite())
    {
        currentStatusWrite = TXFULL;
        currentStatusRead = RXEMPTY;
        serial.write(streamOutBuf, pStreamOutBufPtr - streamOutBuf);
        serial.flush();
        pStreamOutBufPtr = streamOutBuf;
        currentStatusWrite = 0;
        bFlushOutBuffer = false;
        currentStatusRead = 0;
        updateFifoReadAhead();
    }
    // if (serial.availableForWrite())
    //     currentStatusWrite = 0;

    /*     uint8_t status = 0;
        bool bConsumed = false;
        bool bEmpty = true;
        int chRead = 0xFF;
        if (!serial.available())
            status |= RXEMPTY;
        else
        {
            chRead = serial.read();
            bEmpty = false;
        }
        if (!serial.availableForWrite())
            status |= TXFULL;
        while (true)
        {
            uint32_t bits = 0;
            if (bEmpty && serial.available())
            {
                status &= ~RXEMPTY;
                chRead = serial.read();
                bEmpty = false;
            }
            bits = gpio_get_all();
            if ((bits & GPIO_CSR_MASK) == 0)
            {
                if ((bits & GPIO_A0_MASK) == 0)
                {
                    gpio_put_masked(GPIO_CD_MASK, chRead << GPIO_CD7);
                    bConsumed = !bEmpty;//true;
                }
                 else
                    gpio_put_masked(GPIO_CD_MASK, ((uint32_t)status) << GPIO_CD7);

                gpio_set_dir_out_masked(GPIO_CD_MASK);


                while (!gpio_get(GPIO_CSR)) ;
                gpio_set_dir_in_masked(GPIO_CD_MASK);
                if (!bConsumed)
                {
                    if (!serial.available())
                        status |= RXEMPTY;
                    else
                        status &= ~RXEMPTY;

                }
                    else
                        break;
            }
            else if ((bits & GPIO_CSW_MASK) == 0)
            {
                while (!gpio_get(GPIO_CSW)) ;
                uint8_t chWrite = (gpio_get_all() >> GPIO_CD7) & 0xFF;
                if (status & TXFULL != 0)
                    serial.write((uint8_t)chWrite);
                if (!serial.availableForWrite())
                    status |= TXFULL;

            }

        } */
}
