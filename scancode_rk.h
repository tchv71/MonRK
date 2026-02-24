
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

extern bool bPpiKbdMode;
extern uint8_t mouseButtons;
extern int mouseXAbs;
extern int mouseYAbs;
extern int mouseXAbsOld;
extern int mouseYAbsOld;

extern const uint dmaRomSm;
extern void updateTX();

#ifdef __cplusplus
}
#endif

