#include "hardware/pio.h"
// #include "Serial.h"
#include "gpios.h"
#include "fifo.pio.h"
#include "common.h"
#include "hardware/sync.h"
#include "proto.h"
// #include "port_common.h"

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "lwip/arch.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "lwip/apps/sntp.h"

#include "ftpd.h"
#include "ftpd_cpm.h"
#include "pico/multicore.h"
#include "pico/sync.h"
#include "time.h"
#include "stdlib.h"
#include "pico/aon_timer.h"

// extern SerialUSB serial;

const uint8_t RXEMPTY = 1; // MASK FOR RX BUFFER EMPTY
const uint8_t TXFULL = 2;  // MASK FOR TX BUFFER FULL

volatile uint8_t nextValue = 0;           /* pico read-ahead value */
volatile uint8_t currentStatus = RXEMPTY; /* current status register value */

extern "C" void __not_in_flash_func(updateFifoReadAhead)();
/*
 * update the value send to the read PIO
 */
/* inline static */ void __not_in_flash_func(updateFifoReadAhead)()
{
    uint32_t readAhead = nextValue;
    readAhead |= currentStatus << 8;
    readAhead |= 0xFF << 16; // pin direction

    pio_sm_put(FIFO_PIO, fifoReadSm, readAhead);
}

static uint8_t streamInBuf[10 * 1024];
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
    uint32_t writeVal = DMA_PIO->rxf[fifoWrite2Sm];

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
            // currentStatus |= RXEMPTY;
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
    uint32_t readVal = pio_sm_get_blocking(FIFO_PIO, fifoReadSm); // FIFO_PIO->rxf[fifoReadSm];

    if ((readVal & 0x80000000) == 0) // read data (MODE bit==0)
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

/*
 * Set up PIOs for pico <-> CPU interface
 */
void fifoPioInit()
{
    pStreamInBufPtr = pStreamInBufEnd = streamInBuf;
    pStreamOutBufPtr = streamOutBuf;

#if USE_SERIAL_DEBUG
    int fifoReadProgOffset = pio_add_program(FIFO_PIO, &fifoRead_program);
    if (fifoReadProgOffset < 0)
        panic("Failed add fifoReadProgram");

    for (uint i = 0; i < 8; ++i)
    {
        pio_gpio_init(FIFO_PIO, GPIO_CD7 + i);
    }
    pio_gpio_init(FIFO_PIO, DIR);
    pio_sm_set_pins_with_mask(FIFO_PIO, fifoReadSm, DIR_MASK, DIR_MASK);
    pio_sm_set_pindirs_with_mask(FIFO_PIO, fifoReadSm, DIR_MASK, DIR_MASK);
    // pio_sm_set_consecutive_pindirs(FIFO_PIO, fifoReadSm, DIR, 1, true);

    pio_sm_config readFifoConfig = fifoRead_program_get_default_config(fifoReadProgOffset);
    sm_config_set_in_pins(&readFifoConfig, GPIO_CSR);
    sm_config_set_jmp_pin(&readFifoConfig, GPIO_A0);
    sm_config_set_sideset_pin_base(&readFifoConfig, DIR);
    sm_config_set_out_pins(&readFifoConfig, GPIO_CD7, 8);
    sm_config_set_in_shift(&readFifoConfig, true, false, 32);  // R shift
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
    if (fifoWriteProgOffset < 0)
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

#if 0
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
#else
static struct NetInfo
{
    unsigned char ip[4];
} g_net_info =
    {
        //.mac = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x56}, // MAC address
        .ip = {192, 168, 31, 250}, // IP address
                                   //.sn = {255, 255, 255, 0},                    // Subnet Mask
                                   //.gw = {192, 168, 31, 1},                     // Gateway
                                   //.dns = {192, 168, 31, 1},                    // DNS server
                                   //.dhcp = NETINFO_STATIC                       // DHCP enable/disable
};
#endif
// uint8_t s = 0;
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
    void toRTC_Date(const struct tm *t, RTC_DateTypeDef *rt);
}

#define TCP_PORT 1243
#define DEBUG_printf // printf
#define TEST_ITERATIONS 10
#define POLL_TIME_S 5

const uint16_t DATA_BUF_SIZE = sizeof(streamInBuf);
bool bSockEstablished = false;

static TCP_SERVER_T *tcp_server_init(void)
{
    TCP_SERVER_T *state = (TCP_SERVER_T *)calloc(1, sizeof(TCP_SERVER_T));
    if (!state)
    {
        DEBUG_printf("failed to allocate state\n");
        return NULL;
    }
    return state;
}

TCP_SERVER_T *state = 0;

static err_t tcp_server_close(void *arg)
{
    TCP_SERVER_T *state = (TCP_SERVER_T *)arg;
    err_t err = ERR_OK;
    if (state->client_pcb != NULL)
    {
        tcp_arg(state->client_pcb, NULL);
        tcp_poll(state->client_pcb, NULL, 0);
        tcp_sent(state->client_pcb, NULL);
        tcp_recv(state->client_pcb, NULL);
        tcp_err(state->client_pcb, NULL);
        err = tcp_close(state->client_pcb);
        if (err != ERR_OK)
        {
            DEBUG_printf("close failed %d, calling abort\n", err);
            tcp_abort(state->client_pcb);
            err = ERR_ABRT;
        }
        state->client_pcb = NULL;
    }
    if (state->server_pcb)
    {
        tcp_arg(state->server_pcb, NULL);
        tcp_close(state->server_pcb);
        state->server_pcb = NULL;
    }
    return err;
}

static err_t tcp_server_result(void *arg, int status)
{
    TCP_SERVER_T *state = (TCP_SERVER_T *)arg;
    if (status == 0)
    {
        DEBUG_printf("test success\n");
    }
    else
    {
        DEBUG_printf("test failed %d\n", status);
    }
    state->complete = true;
    return tcp_server_close(arg);
}

static err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
    TCP_SERVER_T *state = (TCP_SERVER_T *)arg;
    DEBUG_printf("tcp_server_sent %u\n", len);
    state->sent_len += len;

    return ERR_OK;
}

err_t tcp_server_send_data(void *arg, struct tcp_pcb *tpcb)
{
    TCP_SERVER_T *state = (TCP_SERVER_T *)arg;
    state->sent_len = 0;
    //DEBUG_printf("Writing %ld bytes to client\n", BUF_SIZE);
    // this method is callback from lwIP, so cyw43_arch_lwip_begin is not required, however you
    // can use this method to cause an assertion in debug mode, if this method is called when
    // cyw43_arch_lwip_begin IS needed
    //cyw43_arch_lwip_check();
    uint16_t size = pStreamOutBufPtr - streamOutBuf;
    cyw43_arch_lwip_begin();
    err_t err = tcp_write(tpcb, streamOutBuf, size, TCP_WRITE_FLAG_COPY);
    cyw43_arch_lwip_end();
    if (err != ERR_OK)
    {
        DEBUG_printf("Failed to write data %d\n", err);
        return tcp_server_result(arg, -1);
    }
    return ERR_OK;
}

err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    TCP_SERVER_T *state = (TCP_SERVER_T *)arg;
    if (!p)
    {
        return tcp_server_result(arg, -1);
    }
    // this method is callback from lwIP, so cyw43_arch_lwip_begin is not required, however you
    // can use this method to cause an assertion in debug mode, if this method is called when
    // cyw43_arch_lwip_begin IS needed
    cyw43_arch_lwip_check();
    if (p->tot_len > 0)
    {
        u16_t len = p->tot_len;
        // pStreamInBufEnd = pStreamInBufPtr = streamInBuf;
        if (len > DATA_BUF_SIZE)
            len = DATA_BUF_SIZE;
        ptrdiff_t bufferRemains = (streamInBuf + sizeof(streamInBuf)) - (pStreamInBufEnd + len);
        if (bufferRemains >= 0)
            pbuf_copy_partial(p, pStreamInBufEnd, len, 0);
        else
        {
            pbuf_copy_partial(p, pStreamInBufEnd, len + bufferRemains, 0);
            pbuf_copy_partial(p, streamInBuf, len + bufferRemains, 0);
        }
        // uint32_t ints = save_and_disable_interrupts();
        // pStreamInBufPtr = streamInBuf;
        pStreamInBufEnd = streamInBuf + (pStreamInBufEnd - streamInBuf + len) % sizeof(streamInBuf);
        if (len > 0 && (currentStatus & RXEMPTY) != 0)
        {
            nextValue = *pStreamInBufPtr++;
            currentStatus &= ~RXEMPTY;
            //outLength = 0xFF;
            updateFifoReadAhead();
        }
        // restore_interrupts_from_disabled(ints);
        DEBUG_printf("tcp_server_recv %d/%d err %d\n", p->tot_len, state->recv_len, err);

        tcp_recved(tpcb, p->tot_len);
    }
    pbuf_free(p);
    return ERR_OK;
}

static err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb)
{
    DEBUG_printf("tcp_server_poll_fn\n");
    return tcp_server_result(arg, -1); // no response is an error?
}

static void tcp_server_err(void *arg, err_t err)
{
    if (err != ERR_ABRT)
    {
        DEBUG_printf("tcp_client_err_fn %d\n", err);
        tcp_server_result(arg, err);
    }
}

static err_t tcp_server_accept(void *arg, struct tcp_pcb *client_pcb, err_t err)
{
    TCP_SERVER_T *state = (TCP_SERVER_T *)arg;
    if (err != ERR_OK || client_pcb == NULL)
    {
        DEBUG_printf("Failure in accept\n");
        tcp_server_result(arg, err);
        return ERR_VAL;
    }
    DEBUG_printf("Client connected\n");

    state->client_pcb = client_pcb;
    tcp_arg(client_pcb, state);
    tcp_sent(client_pcb, tcp_server_sent);
    tcp_recv(client_pcb, tcp_server_recv);
    tcp_poll(client_pcb, tcp_server_poll, POLL_TIME_S * 2);
    tcp_err(client_pcb, tcp_server_err);
    bSockEstablished = true;
    return ERR_OK;//tcp_server_send_data(arg, state->client_pcb);
}

static bool tcp_server_open(void *arg)
{
    TCP_SERVER_T *state = (TCP_SERVER_T *)arg;
    DEBUG_printf("Starting server at %s on port %u\n", ip4addr_ntoa(netif_ip4_addr(netif_list)), TCP_PORT);

    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb)
    {
        DEBUG_printf("failed to create pcb\n");
        return false;
    }

    err_t err = tcp_bind(pcb, NULL, TCP_PORT);
    if (err)
    {
        DEBUG_printf("failed to bind to port %u\n", TCP_PORT);
        return false;
    }

    state->server_pcb = tcp_listen_with_backlog(pcb, 1);
    if (!state->server_pcb)
    {
        DEBUG_printf("failed to listen\n");
        if (pcb)
        {
            tcp_close(pcb);
        }
        return false;
    }

    tcp_arg(state->server_pcb, state);
    tcp_accept(state->server_pcb, tcp_server_accept);

    return true;
}

ip4_addr_t sntp_ip;       // SNTP server ip
struct tm timeinfo = {0}; // Struct holding the human format time
time_t unix_time = 0;      // Unix time received from the sntp stack
bool sntp_done = false;

/* function called to receive the unix time from the sntp stack */
void sntp_get_time(uint32_t sec)
{
    unix_time = sec;
    gmtime_r(&unix_time, &timeinfo); // Converts unix time to human format. Unix must be global
}

void sntp_process(void)
{
    if (sntp_enabled() != 1)
    {
        DEBUG_printf("Initializing SNTP\n\r");
        ipaddr_aton("0xC8A007BA", &sntp_ip);     // Set SNTP server IP
        sntp_setoperatingmode(SNTP_OPMODE_POLL); // Set SNTP operation mode to Polling
        sntp_setserver(0, &sntp_ip);             // Set server 0 as the supplied ip address
        sntp_init();                             // Start SNTP process
    }
    if (unix_time != 0)
    {
        DEBUG_printf("SNTP value received from server\n\r");
        aon_timer_set_time_calendar(&timeinfo);
        sntp_stop();
        sntp_done = true;
        unix_time = 0;
    }
}

void networkInit()
{
#if USE_ETHERNET
    if (cyw43_arch_init())
    {
        // printf("failed to initialise\n");
        return;
    }

    cyw43_arch_enable_sta_mode();

    // printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms("Xiaomi_17AE", "rAs48xRq", CYW43_AUTH_WPA2_AES_PSK, 30000))
    {
        // printf("failed to connect.\n");
        return;
    }
    else
    {
        // printf("Connected.\n");
    }
    state = tcp_server_init();
    if (!state)
    {
        return;
    }
    if (!tcp_server_open(state))
    {
        tcp_server_result(state, -1);
        return;
    }

#if 0
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
#endif

#if 0
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
#endif
    ftpd_init(g_net_info.ip);
    // ftpd_cpm_init(g_net_info.ip);

#endif
    /* Get network information */
    // print_network_information(g_net_info);
    //multicore_fifo_push_blocking(0);
    //multicore_fifo_pop_blocking();
}

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
    //if (!sntp_done)
    //    sntp_process();
#if USE_ETHERNET
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
#if 0
    if ((retval = ftpd_cpm_run(g_ftp_buf_cpm)) < 0)
    {
        //printf(" FTP server error : %d\n", retval);

        while (1)
            ;
    }
#endif
#endif
    static uint8_t res;
    static uint16_t len;
#if USE_ETHERNET
    // the following #ifdef is only here so this same example can be used in multiple modes;
    // you do not need it in your code
#if PICO_CYW43_ARCH_POLL
    // if you are using pico_cyw43_arch_poll, then you must poll periodically from your
    // main loop (not from a timer) to check for Wi-Fi driver or lwIP work that needs to be done.
    cyw43_arch_poll();
    // you can poll as often as you like, however if you have nothing else to do you can
    // choose to sleep until either a specified time, or cyw43_arch_poll() has work to do:
    //cyw43_arch_wait_for_work_until(make_timeout_time_ms(1000));
#else
    // if you are not using pico_cyw43_arch_poll, then WiFI driver and lwIP work
    // is done via interrupt in the background. This sleep is just an example of some (blocking)
    // work you might be doing.
    sleep_ms(10);
#endif
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
        uint32_t ints = save_and_disable_interrupts();
        //  if (currentStatus & RXEMPTY)
        //      pio_sm_clear_fifos(FIFO_PIO, fifoReadSm);
        nextValue = *pStreamInBufPtr++;
        currentStatus &= ~RXEMPTY;
        outLength = 0xFF;
        restore_interrupts_from_disabled(ints);
        //updateFifoReadAhead();
    }

    if ((pStreamOutBufPtr == streamOutBuf + sizeof(streamOutBuf) || (bFlushOutBuffer && pStreamOutBufPtr != streamOutBuf)) /* && serial.availableForWrite() */)
    {
        currentStatus |= TXFULL /*  | RXEMPTY */;
        uint16_t len;
        if (!bSockEstablished)
        {
            serial.write(streamOutBuf, pStreamOutBufPtr - streamOutBuf);
            serial.flush();
        }
#if USE_ETHERNET
        else if (bSockEstablished)
        {
            tcp_server_send_data(state, state->client_pcb);
        }
        else
            return;
#endif
        pStreamOutBufPtr = streamOutBuf;
        currentStatus &= ~TXFULL;
        bFlushOutBuffer = false;
        updateFifoReadAhead();
    }
#endif
}
