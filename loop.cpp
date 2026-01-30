#include "hardware/pio.h"
// #include "Serial.h"
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
#include "CircBuffer.h"
#include "init.h"

const uint8_t RXEMPTY = 1; // MASK FOR RX BUFFER EMPTY


CircBuffer<10 * 1024> bufIn;
CircBuffer<1024> bufOut;

static volatile bool bFlushOutBuffer = false;
static volatile uint8_t outLength = 0xff;
extern "C" const uint fifoWrite2Sm;

/*
 * update the value send to the read PIO
 */
static __force_inline bool updateFifoReadAhead()
{
    if (bufIn.isEmpty() || pio_sm_is_tx_fifo_full(FIFO_PIO, fifoReadSm))
        return false;

    uint32_t readAhead = bufIn.getByte(); // nextValue;
    pio_sm_put(FIFO_PIO, fifoReadSm, readAhead);
    return true;
}

/*
 * handle interrupts from the pico<->CPU interface
 */
void __not_in_flash_func(pio_irq_handler_write)()
{
    uint32_t writeVal = DMA_PIO->rxf[fifoWrite2Sm];

    if ((writeVal & (GPIO_A0_MASK >> PIN_CD7)) == 0) // write val
    {
        uint8_t c = writeVal & 0xff;
        bufOut.put(c);
        uint8_t pos = bufOut.getSize();
        if (pos == 2)
            outLength = c + 3;
        if (pos == outLength)
        {
            bFlushOutBuffer = true;
        }
    }
}

extern "C" const uint dmaRomSm;

#ifndef KBD_EMU
volatile uint16_t addr = 0;
static uint16_t lastAddr = 0;
static uint16_t lastAddr2 = 0;
volatile bool bStopRomEmu = false;
volatile bool bEvent = false;
#endif
//volatile uint8_t rowMask = 0xff;
uint32_t __a;
volatile uint8_t __aligned(8) kbdMatr[8] = {255,255,255,255,255,255,255,255};
volatile uint8_t portA = 0;
volatile uint8_t portC = 0xE0; 
volatile uint8_t portB = 0xFF; 


void __not_in_flash_func(pio_irq_handler_rom)()
{
    if (pio_sm_is_rx_fifo_empty(FIFO_PIO, dmaRomSm))
        return;
    uint32_t val = pio_sm_get_blocking(FIFO_PIO, dmaRomSm); // FIFO_PIO->rxf[dmaRomSm];

    bool bWrite = (val & (1 << (PIN_nRD + 4))) != 0;

    uint8_t w_addr = (val & 3);
    if (!bWrite)
    {
        if (w_addr==2)
         updateTX();
        return;
    }
    uint8_t v_val = (val & (GPIO_CD_MASK << 4)) >> (PIN_CD7 + 4);
    switch (w_addr)
    {
#ifdef KBD_EMU
    case 0:
        portA = v_val;
        updateTX();
        break;
    case 2:
        break;
    case 3:
        if ((v_val & 0x80) == 0)
        {
            uint8_t bitNo = (v_val >> 1) & 7;
            if (v_val & 1)
                portC |= 1 << bitNo;
            else
                portC &= ~(1 << bitNo);
        }
        break;

#else
    case 1:
        addr = v_val;
        if (addr == 0x20)
            lastAddr2 = addr;
        else if (addr == 0 && lastAddr2 == 0x20)
        {
            bEvent = true;
            __sev();
        }
        break;
    case 2:
    {
        addr |= ((int16_t)v_val) << 8;
        uint8_t r_val = rom[addr & 0x7f];
        pio_sm_put_blocking(FIFO_PIO, dmaRomSm, 0xFF << 8 | r_val);
    }
    break;
#endif
    default:
        break;
    }
    v55_buf[w_addr] = v_val;
#ifndef KBD_EMU
    if (bStopRomEmu)
        return;
#if 1
    if (addr == 0x44)
    {
        lastAddr = 0x44;
    }
    else if (addr == 0x40)
    {
        lastAddr = (lastAddr == 0x44) ? 0x40 : 0;
    }
    else if (addr == 0)
    {
        if (lastAddr == 0x40)
            bStopRomEmu = true;
        else
            lastAddr = 0;
    }
#endif
#endif
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
#if 0
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
            // pStreamInBufPtr = pStreamInBufEnd = streamInBuf;
            nextValue = 0;
            updateFifoReadAhead();
            return;
        }
    }
    updateFifoReadAhead();
}
#endif

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
#define TIMEZONE 29 // 40 // Korea

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
    uint8_t Calendar_GetDayWeek(RTC_DateTypeDef thisDate);
    void toRTC_Date(const datetime_t *t, RTC_DateTypeDef *rt);
}
void networkInit()
{
    // set_clock_khz();

    // stdio_init_all();
    multicore_fifo_drain();

    wizchip_spi_initialize();
    //gpio_pull_down(PIN_MISO);
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
        // printf(" SNTP failed : %d\n", retval);

        while (1)
            ;
    }

    // printf(" %d-%d-%d, %d:%d:%d\n", time.yy, time.mo, time.dd, time.hh, time.mm, time.ss);

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
    // print_network_information(g_net_info);
    multicore_fifo_pop_blocking();
}

// const uint16_t DATA_BUF_SIZE = sizeof(streamInBuf);
bool bSockEstablished = false;
bool bSerialEstablished = false;

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

void __not_in_flash_func(loop)()
{
    while (updateFifoReadAhead())
        ;
#if USE_ETHERNET
    if (MTX_TRY_ENTER())
    {
        uint8_t retval;
        // mutex_enter_blocking(get_sd_mutex());
        /* Run FTP server */
        if ((retval = ftpd_run(g_ftp_buf)) < 0)
        {
            // printf(" FTP server error : %d\n", retval);

            while (1)
                ;
        }
        // mutex_exit(get_sd_mutex());
        if ((retval = ftpd_cpm_run(g_ftp_buf_cpm)) < 0)
        {
            // printf(" FTP server error : %d\n", retval);

            while (1)
                ;
        }
        MTX_EXIT();
    }
#endif
    static uint8_t res;
    static uint16_t len;
#if USE_ETHERNET
    {
        // MTX_ENTER();
        if (MTX_TRY_ENTER())
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
                while ((len = getSn_RX_RSR(s)) > 0 /*  && pStreamInBufEnd == pStreamInBufPtr */)
                {
                    if (len > bufIn.getMaxSize())
                        len = bufIn.getMaxSize();
                    ptrdiff_t bufferRemains = bufIn.remains(len);
                    if (bufferRemains >= 0)
                    {
                        len = recv(s, bufIn.getEndPtr(), len);
                        bufIn.reserve(len);
                    }
                    else
                    {
                        recv(s, bufIn.getEndPtr(), len + bufferRemains);
                        recv(s, bufIn.getStartPtr(), -bufferRemains);
                        bufIn.setEndToStart(-bufferRemains);
                    }
                    outLength = 0xFF;
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

            default:
                break;
            }
            MTX_EXIT();
        }
        // MTX_EXIT();
    }
#endif
#if USE_SERIAL_DEBUG
    if (!bSockEstablished && serial.available() /* && ((currentStatus & TXFULL) == 0) */ /* && pStreamInBufPtr == pStreamInBufEnd */)
    {
        bSerialEstablished = true;
        while (serial.available())
        {
            bufIn.put(serial.read());
        }
        outLength = 0xFF;
    }

    if (bFlushOutBuffer && bufOut.getSize() > 0)
    {
        uint16_t len;
        if (!bSockEstablished && !bSerialEstablished)
        {
            bufOut.clear();
        }
        else if (!bSockEstablished)
        {
            serial.write(bufOut.getPtr(), bufOut.getSize());
            serial.flush();
            bufOut.setCurToEnd();
        }
#if USE_ETHERNET
        else if (bSockEstablished && (len = getSn_TX_FSR(s)) > 0)
        {
            //recursive_mutex_enter_blocking(get_sd_mutex());
            if (bufOut.getPtr() > bufOut.getEndPtr())
            {
                // Block crosses buffer boundary
                send(s, bufOut.getPtr(), bufOut.curToEnd());
                send(s, bufOut.getStartPtr(), bufOut.endToStart());
            }
            else
            {
                uint16_t size = bufOut.getSize();
                if (len > size)
                    len = size;
                send(s, bufOut.getPtr(), len);
            }
            bufOut.setCurToEnd();
            //recursive_mutex_exit(get_sd_mutex());
        }
#endif
        bFlushOutBuffer = false;
    }
#endif
}
