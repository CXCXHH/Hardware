#ifndef __MAIXCAM_H__
#define __MAIXCAM_H__

#include <stdint.h>
#include "board.h" 

/*================================================================================*/
// 宏定义
/*================================================================================*/
#define MAIXCAM_RX_BUFF_SIZE   4 

/*================================================================================*/
// 对外公开的函数声明
/*================================================================================*/
void MaixCAM_Init(void);
uint8_t Get_MaixCAM_Direction(void);
uint8_t Get_MaixCAM_Expected_Number(void);
void MaixCAM_Send_Command(uint8_t command);
uint32_t Get_MaixCAM_Frame_Counter(void);
uint8_t Get_MaixCAM_Last_Byte(void);
void MaixCAM_INST_IRQHandler(void);

#endif // __MAIXCAM_H__
