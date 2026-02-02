
#pragma once

#include "inttypes.h"
#include "gpios.h"

#ifdef __cplusplus
extern "C" {
#endif
void rk_keyboard (const hid_keyboard_report_t *report, uint16_t len);
void rk_scancode_s(uint8_t code);
extern volatile uint8_t kbdMatr[9];// = {255,255,255,255,255,255,255,255};
extern volatile uint8_t portA;
extern volatile uint8_t portB; 
extern volatile uint8_t portC; 
extern const uint dmaRomSm;
void __force_inline __not_in_flash_func(updateTX)()
{
    //while (!pio_sm_is_tx_fifo_empty(FIFO_PIO, dmaRomSm))
    //uint32_t val = FIFO_PIO->txf[dmaRomSm];
    //portA = 0x7F;
    portB = 0xff;
    // if (portA==0)
    // {
    //     portB = kbdMatr[0] & kbdMatr[1] & kbdMatr[2] & kbdMatr[3] & kbdMatr[4] & kbdMatr[5] & kbdMatr[6] & kbdMatr[7];
    // }
    // else
    {
        uint8_t mask = 1;
        for (uint8_t i = 0; i < 8; ++i)
        {
            if (!(portA & mask))
                portB &= kbdMatr[i];
            mask = mask << 1;
        }
    }
    //portC &= ~0x20;
    pio_sm_clear_fifos(FIFO_PIO, dmaRomSm);

    pio_sm_put_blocking(FIFO_PIO, dmaRomSm, ((uint32_t)portC << 24) | ((uint32_t)portB << 16) | ((uint32_t)portA<<8) | 0xFF);
}

#ifdef __cplusplus
}
#endif

