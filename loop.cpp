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

/*
 * update the value send to the read PIO
 */
/* static __force_inline */ bool updateFifoReadAhead()
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
    uint32_t writeVal = pio_sm_get_blocking(DMA_PIO, fifoWrite2Sm);//DMA_PIO->rxf[fifoWrite2Sm];

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

const uint8_t PPI_MOUSE_MODE = 0x80;
const uint8_t PPI_KBD_MODE = 0x8A;

#ifdef MSX
volatile uint8_t kbdMatr[16] = {255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255};
#else
volatile uint8_t kbdMatr[9] = {255,255,255,255,255,255,255,255,255};
#endif
volatile uint8_t portA = 0xFF;
volatile uint8_t portB = 0xFF;
volatile uint8_t portC = 0xE0; 
volatile uint8_t portCtrl = 0x8A; 

volatile uint8_t priSlots = 0xFF;
volatile uint8_t extSlots = 0;
// SlotID in F000EEPP format for pages 0-3
volatile uint8_t extSlotsTbl[4] = {0};

bool bProgDc_inited = false;


void __no_inline_not_in_flash_func(port_set_addr)(uint8_t addr)
{
    gpio_put(PIN_DIR, 0);
    gpio_set_dir_out_masked(GPIO_CD_MASK);
    gpio_put_masked(GPIO_CD_MASK, ((int)addr) << PIN_CD7);
    gpio_put(PIN_ADDRWR, 1);
    //__asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");
    sleep_us(1);
    gpio_put(PIN_ADDRWR, 0);
}


uint8_t __not_in_flash_func(port_read)(uint8_t addr)
{
    port_set_addr(addr);
    gpio_put(PIN_DIR, 1);
    gpio_set_dir_in_masked(GPIO_CD_MASK);
    gpio_put(PIN_PDC_nIOR, 0);
    //__asm volatile("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");
    sleep_us(1);
    uint8_t val = (gpio_get_all() >> PIN_CD7) & 0xFF;
    gpio_set_dir_out_masked(GPIO_CD_MASK);
    gpio_put(PIN_PDC_nIOR, 1);
    gpio_put(PIN_DIR, 0);
    return val;
}

void  __not_in_flash_func(port_write)(uint8_t addr, uint8_t val)
{
    port_set_addr(addr);
    gpio_put_masked(GPIO_CD_MASK, ((int)val) << PIN_CD7);
    gpio_put(PIN_PDC_nIOW, 0);
    //__asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");
    sleep_us(1);
    gpio_put(PIN_PDC_nIOW, 1);
}

volatile uint8_t val;
volatile uint8_t val1;

void __no_inline_not_in_flash_func(progDc_init)()
{
    if (bProgDc_inited)
        return;
    pio_sm_set_enabled(DMA_PIO, ffffWriteSm, false);
    pio_sm_set_enabled(DMA_PIO, dmaRomWrSm, false);
    gpio_init_mask(HOLD2_MASK | HLDA_MASK | PDC_nIOR_MASK | PDC_nIOW_MASK | ADDRWR_MASK | DIR_MASK | GPIO_CD_MASK);
    gpio_put(PIN_HOLD2, 1);
    gpio_put(PIN_PDC_nIOR, 1);
    gpio_put(PIN_PDC_nIOW, 1);
    gpio_put(PIN_ADDRWR, 0);
    gpio_put(PIN_DIR, 0);
    //gpio_put_masked(GPIO_CD_MASK, 0xFF <<  PIN_CD7);


    gpio_set_dir_out_masked(PDC_nIOR_MASK | PDC_nIOW_MASK | ADDRWR_MASK | HOLD2_MASK | DIR_MASK | GPIO_CD_MASK);

    while (gpio_get(PIN_HLDA)) ; // Wait other DMA cycle to stop
    gpio_set_dir(PIN_HOLD2, true);
    while (!gpio_get(PIN_HLDA)) ;

    val = port_read(0xFF);
    port_write(0xFF, 0xA0);

    bProgDc_inited = true;
}

void setupFifoGpio();

void __not_in_flash_func(progDc_deinit)()
{
    if (!bProgDc_inited)
        return;

    val1 = port_read(0xFF);
    port_write(0xFF, val); // Working mode

    gpio_put(PIN_DIR, 1);
    gpio_put(PIN_ADDRWR, 0);
    gpio_put(PIN_HOLD2, 0);
    while (gpio_get(PIN_HLDA)) ;
    gpio_set_dir_in_masked(PDC_nIOR_MASK | PDC_nIOW_MASK | GPIO_CD_MASK | ADDRWR_MASK);
    //pio_gpio_init(FIFO_PIO, PIN_DIR);
    setupFifoGpio();
    pio_sm_set_pins_with_mask(DMA_PIO, ffffWriteSm, DIR_MASK, DIR_MASK);

    pio_sm_restart(DMA_PIO, ffffWriteSm);
    pio_sm_set_enabled(DMA_PIO, ffffWriteSm, true);
    pio_sm_set_enabled(DMA_PIO, dmaRomWrSm, true);
   
    bProgDc_inited = false;
}

void __no_inline_not_in_flash_func(setupMMap)(uint8_t oSlt, uint8_t sltSel)
{
    uint8_t oPriS = oSlt;
    uint8_t priS = priSlots;
    uint8_t extS = extSlots;
    for (uint8_t i = 0; i < 4; ++i)
    {
        uint8_t otbl = extSlotsTbl[i];
        bool bPriSlotChanged = (oPriS & 3) != (priS & 3);
        if ( bPriSlotChanged || (otbl & 3) == sltSel)
        {
            uint8_t curSltId = ((extS & 3) << 2) | (priS & 3) | 0x80;
            extSlotsTbl[i] = curSltId;
            if (bPriSlotChanged ||  otbl != extSlotsTbl[i])
            {
                progDc_init();
                port_write(0, 4);
                break;
                // Slot changed
                if ((curSltId & 0x7f) == 0)
                {
                    // Slot 0 - MAIN ROM 32k
                    for (uint8_t j = 0; j<0x40; ++j)
                        port_write(j + i * 0x40, 3);
                }
                else if (curSltId == 0x83)
                {
                    // Slot 3-0
                    uint8_t val = 5 + i * 0x40 + (i&1)*0x10;
                    if (i>=2)
                      val += 8;
                    for (uint8_t j = 0; j<0x40; ++j)
                        port_write(j + i * 0x40, val);
                } 
            }
        }
        oPriS >>= 2; 
        priS >>= 2;
        extS >>= 2;
    }
    progDc_deinit();
}

void __not_in_flash_func(pio_irq_handler_ffff_write)()
{
    if (pio_sm_is_rx_fifo_empty(DMA_PIO, ffffWriteSm))
         return;

    uint32_t writeVal = pio_sm_get_blocking(DMA_PIO, ffffWriteSm); // DMA_PIO->rxf[fifoWrite2Sm];

    extSlots = (writeVal >> (PIN_CD7 - PIN_nFFFF_W)) & 0xff;
    uint8_t sltSel = (extSlots & 0xC0) >> 6;
    pio_sm_put(FIFO_PIO, ffffReadSm, 0xFF << 8 | ~extSlots);
    setupMMap(priSlots, sltSel);
}


uint8_t portASave = portA;
uint8_t portBSave = portB;
uint8_t portCSave = portC; 

uint8_t mouseButtons = 0;
int mouseXAbs = 0;
int mouseYAbs = 0;
int mouseXAbsOld = 0;
int mouseYAbsOld = 0;
const int mouseDiv = 1;//10;
//const uint8_t ADDR_SHIFT = 0;

bool bPpiKbdMode = true;

void __no_inline_not_in_flash_func (updateTX)()
{
    pio_sm_clear_fifos(FIFO_PIO, fifoRomRdSm);
    if (!bPpiKbdMode)
    {
        int diffX = (mouseXAbs - mouseXAbsOld) / mouseDiv;
        //if (diffX > 127) diffX = 127; else if (diffX<-127) diffX = -127;
        int diffY = (mouseYAbs - mouseYAbsOld) / mouseDiv; 
        //if (diffY > 127) diffY = 127; else if (diffY<-127) diffY = -127;
        portA = diffX;
        portB = diffY;
        portC = mouseButtons;
    }
    else
    {
#ifdef MSX
        uint8_t row = portC & 0xF;
        portB = kbdMatr[row];
#else
        portB = 0xff;
        uint8_t mask = 1;
        for (uint8_t i = 0; i < 8; ++i)
        {
            if (!(portA & mask))
                portB &= kbdMatr[i];
            mask = mask << 1;
        }
#endif
    }

    pio_sm_put_blocking(FIFO_PIO, fifoRomRdSm, ((uint32_t)portCtrl << 24) |((uint32_t)portC << 16) | ((uint32_t)portB << 8) | (uint32_t)portA);
}

void __not_in_flash_func(pio_irq_handler_rom_wr)()
{
    if (pio_sm_is_rx_fifo_empty(DMA_PIO, dmaRomWrSm))
    {
        return;
    }
    uint32_t val = pio_sm_get_blocking(DMA_PIO, dmaRomWrSm);
    DMA_PIO->irq = 1;

    uint8_t w_addr = (val & 3);
    uint8_t v_val = (val & (GPIO_CD_MASK << 4)) >> (PIN_CD7 + 4);
    switch (w_addr)
    {
#if 1//def KBD_EMU
    case 0:
    {
        uint8_t oSlt = priSlots;
        priSlots = portA = v_val;
        setupMMap(oSlt, 0);
        updateTX();
        break;
    }
    case 1:
        portB = v_val;
        break;
    case 2:
        portC = v_val;
        break;
    case 3:
        portCtrl = v_val;
        if (v_val == PPI_MOUSE_MODE)
        {
            portASave = portA; portBSave = portB; portCSave = portC;
            //portA = portB = portC = 0;

            bPpiKbdMode = false;
            updateTX();
        }
        else if (v_val == PPI_KBD_MODE)
        {
            portA = portASave; portB = portBSave; portC = portCSave;
            bPpiKbdMode = true;
            updateTX();
        }
        if ( bPpiKbdMode && (v_val & 0x80) == 0)
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
}

void __not_in_flash_func(pio_irq_handler_rom_rd)()
{
    if (pio_sm_is_rx_fifo_empty(FIFO_PIO, fifoRomRdSm))
        return;
    uint32_t val = pio_sm_get_blocking(FIFO_PIO, fifoRomRdSm);
    uint8_t w_addr = (val & 3);
    if (!bPpiKbdMode)
    {
        if (w_addr == 0)
            mouseXAbsOld = mouseXAbs - mouseXAbs % mouseDiv;
        else if (w_addr == 1)
            mouseYAbsOld = mouseYAbs - mouseYAbs % mouseDiv;
        else
            return;
        updateTX();
        return;
    }
    if (w_addr == 2)
        updateTX();
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
#define TIMEZONE 29 // 40 // Korea

/* SNTP */
static uint8_t g_sntp_buf[ETHERNET_BUF_MAX_SIZE] = {
    0,
};
static uint8_t g_sntp_server_ip[4] = {216, 239, 35, 8}; // time.google.com

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
    do
    {
        wizchip_initialize();
    } while (!wizchip_check());
    
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

        // while (1)
        //     ;
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
#ifdef USE_SERIAL_DEBUG
    if (!bSockEstablished && serial.available() /* && ((currentStatus & TXFULL) == 0) */ /* && pStreamInBufPtr == pStreamInBufEnd */)
    {
        bSerialEstablished = true;
        while (serial.available())
        {
            bufIn.put(serial.read());
        }
        outLength = 0xFF;
    }

#endif
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
}
