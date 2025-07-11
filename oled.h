#ifndef __OLED_H
#define __OLED_H 

#include "board.h"
#include "stdlib.h"	

//-----------------OLED端口定义----------------

#define OLED_RES_Clr()  DL_GPIO_clearPins(OLED_PORT,OLED_RES_PIN)//RES
#define OLED_RES_Set()  DL_GPIO_setPins(OLED_PORT,OLED_RES_PIN)

#define OLED_DC_Clr()   DL_GPIO_clearPins(OLED_PORT,OLED_DC_PIN)//DC
#define OLED_DC_Set()   DL_GPIO_setPins(OLED_PORT,OLED_DC_PIN)

#define OLED_CS_Clr()   DL_GPIO_clearPins(OLED_PORT,OLED_CS_PIN)//CS
#define OLED_CS_Set()   DL_GPIO_setPins(OLED_PORT,OLED_CS_PIN)


#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据

void OLED_ClearPoint(u8 x,u8 y);
void OLED_ColorTurn(u8 i);
void OLED_DisplayTurn(u8 i);
void OLED_WR_Byte(u8 dat,u8 mode);
void OLED_DisPlay_On(void);
void OLED_DisPlay_Off(void);
void OLED_Refresh(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_DrawLine(u8 x1,u8 y1,u8 x2,u8 y2,u8 mode);
void OLED_DrawCircle(u8 x,u8 y,u8 r);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size1,u8 mode);
void OLED_ShowChar6x8(u8 x,u8 y,u8 chr,u8 mode);
void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 size1,u8 mode);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size1,u8 mode);
void OLED_ShowChinese(u8 x,u8 y,u8 num,u8 size1,u8 mode);
void OLED_ScrollDisplay(u8 num,u8 space,u8 mode);
void OLED_ShowPicture(u8 x,u8 y,u8 sizex,u8 sizey,u8 BMP[],u8 mode);
void OLED_Init(void);

void OLED_Update(void);
void OLED_Show_String(uint8_t han,uint8_t lie,uint8_t *string);
void OLED_DisplayLine(uint8_t han,uint8_t lie,uint8_t *temp);
void OLED_ShowBinNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size1,uint8_t mode);
void OLED_ShowSignedNum(uint8_t x,uint8_t y,int32_t num,uint8_t len,uint8_t size1,uint8_t mode);
void OLED_ShowFloatNum(uint8_t x,uint8_t y,double num,uint8_t intlen,uint8_t fralen,uint8_t size1,uint8_t mode);
void OLED_ShowString_Horizontal(unsigned char x, unsigned char y, unsigned char *p);

#endif

