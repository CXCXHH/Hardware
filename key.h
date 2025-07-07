#ifndef __KEY_H
#define __KEY_H

#include "main.h"

#define KEY_Number 4 // 按键数量
#define KEY_LONG_PRESS_TIME  70  // 长按的最大间隔 70 * 10ms = 700ms
#define KEY_DOUBLE_CLICK_INTERVAL 35  // 两次单击的最大间隔 35 * 10ms = 350ms

#define LED1_ON()       DL_GPIO_clearPins(LED_PORT, LED_LED1_PIN)
#define LED1_OFF()      DL_GPIO_setPins(LED_PORT, LED_LED1_PIN)
#define LED1_TOGGLE()   DL_GPIO_togglePins(LED_PORT, LED_LED1_PIN)

#define KEY1_STATE (DL_GPIO_readPins(KEY_KEY1_PORT, KEY_KEY1_PIN) ==0)?0:1// 按键1状态
#define KEY2_STATE (DL_GPIO_readPins(KEY_KEY2_PORT, KEY_KEY2_PIN) ==0)?0:1// 按键2状态
#define KEY3_STATE (DL_GPIO_readPins(KEY_KEY3_PORT, KEY_KEY3_PIN) ==0)?0:1// 按键3状态
#define KEY4_STATE (DL_GPIO_readPins(KEY_KEY4_PORT, KEY_KEY4_PIN) ==0)?0:1// 按键4状态

typedef struct 
{
    bool Down_State; // 按键按下瞬间状态
    bool Short_Flag; // 短按标志
    bool Long_Flag;  // 长按标志
    bool Double_Flag; // 双击标志
    bool Double_Time_EN;     // 双击间隔计时使能
    uint8_t Down_Time; // 按下时间
    uint8_t Double_Time; // 双击时间
    uint8_t Judge_State; // 按键判断时间
} KEY;

extern KEY key[KEY_Number];
void Key_Read(void);
#endif
