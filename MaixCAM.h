#ifndef __MAIXCAM_H__
#define __MAIXCAM_H__

#include <stdint.h>
#include "board.h" 

/*================================================================================*/
// 宏定义
/*================================================================================*/

// Python发送: [0x5B, 0x5B, digit_value, 0xB3] 共4个字节
#define MAIXCAM_RX_BUFF_SIZE   4 

/*================================================================================*/
// 对外公开的函数声明
/*================================================================================*/

/**
 * @brief  初始化与MaixCAM通信的串口中断
 * @param  None
 * @retval None
 * 
 */
void MaixCAM_Init(void);

/**
 * @brief  获取MaixCAM解析后的坐标数据
 * @param  None
 * @retval uint8_t 坐标数据
 */
uint8_t Get_MaixCAM_Coordinate(void);

/**
 * @brief  获取MaixCAM解析后的识别数字
 * @param  None
 * @retval uint8_t 识别出的数字
 */
uint8_t Get_MaixCAM_Number(void);

/**
 * @brief  手动清除内部存储的识别数字
 * @note   用于在任务开始时清除旧的、可能残留的数字，避免误判
 * @param  None
 * @retval None
 */
void MaixCAM_Clear_Number(void);


// 中断服务函数原型声明
void MaixCAM_INST_IRQHandler(void);

#endif // __MAIXCAM_H__
