#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "main.h"

/*编码器端口读取宏定义*/                                                                 
#define Read_Encoder_A 	(DL_GPIO_readPins(Encoder_PORT,Encoder_A_PIN)==Encoder_A_PIN)?1:0//左轮 A相
#define Read_Encoder_B  (DL_GPIO_readPins(Encoder_PORT,Encoder_B_PIN)==Encoder_B_PIN)?1:0//左轮 B相
#define Read_Encoder_C 	(DL_GPIO_readPins(Encoder_PORT,Encoder_C_PIN)==Encoder_C_PIN)?1:0//右轮 A相
#define Read_Encoder_D 	(DL_GPIO_readPins(Encoder_PORT,Encoder_D_PIN)==Encoder_D_PIN)?1:0//右轮 B相

#define DISTANCE_COUNT      (0.05535f)  // 单位: cm/count
// 测速时间间隔 (秒)
#define SPEED_CALC_INTERVAL_S   (0.01f)      // 10ms
extern float Motor1_Speed;
extern float Motor2_Speed;

void Encoder_Speed(void);
float Encoder_AB_Distance(void);
float Encoder_CD_Distance(void);
void Move_With_PID(float distance_cm, float speed_cm_s);

#endif
