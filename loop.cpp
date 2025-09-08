#include "hardware/pio.h"
//#include "Serial.h"
#include "gpios.h"
#include "fifo.pio.h"
#include "common.h"
#include "hardware/sync.h"
#include "proto.h"
#include "port_common.h"

#include "socket.h"
#include "wizchip_conf.h"
extern "C"
{
#include "w5x00_spi.h"
#include "timer.h"
#include <time.h>
}
#include "sntp.h"
#include "hardware/rtc.h"

#include "ftpd.h"
#include "ftpd_cpm.h"
#include "pico/multicore.h"
#include "pico/sync.h"

//extern SerialUSB serial;

const uint8_t RXEMPTY = 1; // MASK FOR RX BUFFER EMPTY
const uint8_t TXFULL = 2;  // MASK FOR TX BUFFER FULL

volatile uint8_t nextValue = 0;         /* pico read-ahead value */
volatile uint8_t currentStatus = RXEMPTY; /* current status register value */


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

//     1   1   2   2   3   3   0   0
//     0   0   0   0   0   0   1   1
// B   _______|_______|_______|_______|_______|_______|

// MODE_ 1 _ 0 _ 1 _ 0 _ 1 _ 0 _ 1 _ 0 _ 1 _ 0 _ 1 _ 0 _
// RD   |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_|

//         1       2       3       -       -
// D   -------+-------+-------+-------+-------+-------+--

//         0       0       0      1
// RXE -------+-------+-------+-------+-------+-------+--
//

void __not_in_flash_func(pio_irq_handler_read)()
{
    uint32_t readVal = pio_sm_get_blocking(FIFO_PIO, fifoReadSm);//FIFO_PIO->rxf[fifoReadSm];

    if (!(readVal &  0x80000000)) // read data (MODE bit==0)
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
            currentStatus = currentStatus | RXEMPTY;
            //pStreamInBufPtr = pStreamInBufEnd = streamInBuf;
            nextValue = 0;
            updateFifoReadAhead();
            return;
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

#if USE_SERIAL_DEBUG
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
#endif
}

/* Clock */
#define PLL_SYS_KHZ (133 * 1000)
/* Clock */
static void set_clock_khz(void)
{
    // set a system clock frequency in khz
    set_sys_clock_khz(PLL_SYS_KHZ, true);

    // configure the specified clock
    clock_configure(
        clk_peri,
        0,                                                // No glitchless mux
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, // System PLL on AUX mux
        PLL_SYS_KHZ * 1000,                               // Input frequency
        PLL_SYS_KHZ * 1000                                // Output (must be same as no divider)
    );
}
/* Network */
static wiz_NetInfo g_net_info =
{
    .mac = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x56}, // MAC address
    .ip = {192, 168, 31, 250},                   // IP address
    .sn = {255, 255, 255, 0},                    // Subnet Mask
    .gw = {192, 168, 31, 1},                     // Gateway
    .dns = {192, 168, 31, 1},                    // DNS server
    .dhcp = NETINFO_STATIC                       // DHCP enable/disable
};

uint8_t s = 0;
const uint16_t SOCKET_PORT = 1243;

/* Buffer */
#define ETHERNET_BUF_MAX_SIZE (1024 * 16)
/* Socket */
#define SOCKET_SNTP 1

/* Timeout */
#define RECV_TIMEOUT (1000 * 10) // 10 seconds

/* Timezone */
#define TIMEZONE 29 //40 // Korea


/* SNTP */
static uint8_t g_sntp_buf[ETHERNET_BUF_MAX_SIZE] = {
    0,
};
static uint8_t g_sntp_server_ip[4] = {216, 239, 35, 0}; // time.google.com

/* Timer */
static volatile uint32_t g_msec_cnt = 0;

/* Timer */
static void repeating_timer_callback(void)
{
    g_msec_cnt++;
}

static time_t millis(void)
{
    return g_msec_cnt;
}


/* FTP */
static uint8_t g_ftp_buf[ETHERNET_BUF_MAX_SIZE] = {
    0,
};
static uint8_t g_ftp_buf_cpm[ETHERNET_BUF_MAX_SIZE] = {
    0,
};

extern "C"
{
uint8_t Calendar_GetDayWeek (RTC_DateTypeDef thisDate);
void toRTC_Date(const  datetime_t * t, RTC_DateTypeDef *rt);
}
void networkInit()
{
    //set_clock_khz();

    //stdio_init_all();
    multicore_fifo_drain();

    wizchip_spi_initialize();
    wizchip_cris_initialize();

    wizchip_reset();
    wizchip_initialize();
    wizchip_check();

    network_initialize(g_net_info);
    uint8_t res = socket(s, Sn_MR_TCP, SOCKET_PORT, 0);

    wizchip_1ms_timer_initialize(repeating_timer_callback);

    network_initialize(g_net_info);

    SNTP_init(SOCKET_SNTP, g_sntp_server_ip, TIMEZONE, g_sntp_buf);
    uint8_t retval = 0;
    uint32_t start_ms = millis();
    datetime time;

    /* Get time */
    do
    {
        retval = SNTP_run(&time);

        if (retval == 1)
        {
            break;
        }
    } while ((millis() - start_ms) < RECV_TIMEOUT);

    if (retval != 1)
    {
        //printf(" SNTP failed : %d\n", retval);

        while (1)
            ;
    }

    //printf(" %d-%d-%d, %d:%d:%d\n", time.yy, time.mo, time.dd, time.hh, time.mm, time.ss);

    datetime_t t;
    t.day = time.dd;
    t.year = time.yy;
    t.month = time.mo;
    t.hour = time.hh;
    t.min = time.mm;
    t.sec = time.ss;
    RTC_DateTypeDef sDate;
    toRTC_Date(&t, &sDate);
    t.dotw = Calendar_GetDayWeek(sDate);
     // Start the RTC
    rtc_init();
    rtc_set_datetime(&t);

    ftpd_init(g_net_info.ip);
    ftpd_cpm_init(g_net_info.ip);
    

    /* Get network information */
    //print_network_information(g_net_info);
    multicore_fifo_pop_blocking();
}

const uint16_t DATA_BUF_SIZE = sizeof(streamInBuf);
bool bSockEstablished = false;

class SerialUSB
{
    int m_char = -1;

public:
    bool available()
    {
        if (m_char >= 0)
            return true;
        m_char = stdio_getchar_timeout_us(0);
        return m_char >= 0;
    }
    uint8_t read()
    {
        if (m_char >= 0)
        {
            int ret = m_char;
            m_char = -1;
            return ret;
        }
        return stdio_getchar();
    }
    size_t write(uint8_t c)
    {
        stdio_putchar(c);
        return 1;
    }
    size_t write(const uint8_t *buffer, size_t size)
    {
        size_t n = 0;
        while (size--)
        {
            if (write(*buffer++))
                n++;
            else
                break;
        }
        return n;
    }
    void flush()
    {
        stdio_flush();
    }
} serial;

void loop()
{
    uint8_t retval;
#if USE_ETHERNET
    //mutex_enter_blocking(get_sd_mutex());
    /* Run FTP server */
    if ((retval = ftpd_run(g_ftp_buf)) < 0)
    {
        //printf(" FTP server error : %d\n", retval);

        while (1)
            ;
    }
    //mutex_exit(get_sd_mutex());
    if ((retval = ftpd_cpm_run(g_ftp_buf_cpm)) < 0)
    {
        //printf(" FTP server error : %d\n", retval);

        while (1)
            ;
    }
#endif
    static uint8_t res;
    static uint16_t len;
#if USE_ETHERNET
    {
        switch ((res = getSn_SR(s)))
        {
        case SOCK_ESTABLISHED:
            // Interrupt clear
            if (getSn_IR(s) & Sn_IR_CON)
            {
                setSn_IR(s, Sn_IR_CON);
            }
            bSockEstablished = true;
            while ((len = getSn_RX_RSR(s)) > 0/*  && pStreamInBufEnd == pStreamInBufPtr */)
            {
                //pStreamInBufEnd = pStreamInBufPtr = streamInBuf;
                if (len > DATA_BUF_SIZE)
                    len = DATA_BUF_SIZE;
                ptrdiff_t bufferRemains = (streamInBuf + sizeof(streamInBuf)) - (pStreamInBufEnd + len);
                if (bufferRemains >= 0)
                    len = recv(s, pStreamInBufEnd, len);
                else
                {
                    recv(s, pStreamInBufEnd, len + bufferRemains);
                    recv(s, streamInBuf, -bufferRemains);
                }
                //uint32_t ints = save_and_disable_interrupts();
                //pStreamInBufPtr = streamInBuf;
                pStreamInBufEnd = streamInBuf + (pStreamInBufEnd - streamInBuf + len) % sizeof(streamInBuf);
                if (len>0 && (currentStatus & RXEMPTY) != 0)
                {
                    nextValue = *pStreamInBufPtr++;
                    currentStatus &= ~RXEMPTY;
                    outLength = 0xFF;
                    updateFifoReadAhead();
                }
                //restore_interrupts_from_disabled(ints);
            }
            break;
		case SOCK_CLOSE_WAIT:
			disconnect(s);
            bSockEstablished = false;
			break;

		case SOCK_CLOSED:
            socket(s, Sn_MR_TCP, SOCKET_PORT, 0);
			break;

		case SOCK_INIT:
			listen(s);
			break;

		case SOCK_LISTEN:
			break;

		default :
			break;
        
        }

    }
#endif
#if USE_SERIAL_DEBUG
    if (!bSockEstablished && serial.available() && ((currentStatus & TXFULL) == 0) /* && pStreamInBufPtr == pStreamInBufEnd */)
    {
        if (pStreamInBufEnd == pStreamInBufPtr)
            pStreamInBufEnd = pStreamInBufPtr = streamInBuf;
        while (serial.available() && pStreamInBufEnd != streamInBuf + sizeof(streamInBuf))
        {
            *pStreamInBufEnd++ = serial.read();
        }
        //uint32_t ints = save_and_disable_interrupts();
        // if (currentStatus & RXEMPTY)
        //     pio_sm_clear_fifos(FIFO_PIO, fifoReadSm);
        nextValue = *pStreamInBufPtr++;
        currentStatus &= ~RXEMPTY;
        outLength = 0xFF;
        //restore_interrupts_from_disabled(ints);
        updateFifoReadAhead();
    }

    if ((pStreamOutBufPtr == streamOutBuf + sizeof(streamOutBuf) || (bFlushOutBuffer && pStreamOutBufPtr != streamOutBuf)) /* && serial.availableForWrite() */)
    {
        currentStatus |= TXFULL/*  | RXEMPTY */;
        uint16_t len;
        if (!bSockEstablished)
        {
            serial.write(streamOutBuf, pStreamOutBufPtr - streamOutBuf);
            serial.flush();
        }
#if USE_ETHERNET
        else if (bSockEstablished && (len = getSn_TX_FSR(s))>0)
        {
            uint16_t size = pStreamOutBufPtr - streamOutBuf;
            if (len > size)
              len = size;
            send(s, streamOutBuf, len);
        }
        else return;
#endif
        pStreamOutBufPtr = streamOutBuf;
        currentStatus &= ~TXFULL;
        bFlushOutBuffer = false;
        updateFifoReadAhead();
    }
#endif
}
