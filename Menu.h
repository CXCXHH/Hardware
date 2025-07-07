#ifndef __MENU_H
#define __MENU_H 

#include <stdint.h>

#define BEEP_ON()       DL_GPIO_setPins(BEEP_PORT, BEEP_PIN_0_PIN)
#define BEEP_OFF()      DL_GPIO_clearPins(BEEP_PORT, BEEP_PIN_0_PIN)
#define BEEP_Toggle()   DL_GPIO_togglePins(BEEP_PORT, BEEP_PIN_0_PIN)

void Menu_Init(void);
void Menu_loop(void);

#endif