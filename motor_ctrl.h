#ifndef __MOTOR_CTRL_H__
#define __MOTOR_CTRL_H__

#include "main.h"

#define AIN1(x)   x?DL_GPIO_setPins(Motor_Ctrl_PORT, Motor_Ctrl_AIN1_PIN):DL_GPIO_clearPins(Motor_Ctrl_PORT, Motor_Ctrl_AIN1_PIN)
#define AIN2(x)   x?DL_GPIO_setPins(Motor_Ctrl_PORT, Motor_Ctrl_AIN2_PIN):DL_GPIO_clearPins(Motor_Ctrl_PORT, Motor_Ctrl_AIN2_PIN)
#define BIN1(x)   x?DL_GPIO_setPins(Motor_Ctrl_PORT, Motor_Ctrl_BIN1_PIN):DL_GPIO_clearPins(Motor_Ctrl_PORT, Motor_Ctrl_BIN1_PIN)
#define BIN2(x)   x?DL_GPIO_setPins(Motor_Ctrl_PORT, Motor_Ctrl_BIN2_PIN):DL_GPIO_clearPins(Motor_Ctrl_PORT, Motor_Ctrl_BIN2_PIN)

#define Motor1_Forward()	{AIN1(1);AIN2(0);}
#define Motor1_Backward()	{AIN1(0);AIN2(1);}
#define Motor1_Stop()		{AIN1(0);AIN2(0);}

#define Motor2_Forward()	{BIN1(0);BIN2(1);}
#define Motor2_Backward()	{BIN1(1);BIN2(0);}
#define Motor2_Stop()		{BIN1(0);BIN2(0);}

// 角度误差阈值
#define ANGLE_ERROR_THRESHOLD 0.5f 
// 转弯最大角速度
#define MAX_TURN_ANGULAR_SPEED 50.0f 

void Set_Motor1_PWm(int Target_PWM);
void Set_Motor2_PWm(int Target_PWM);
void Set_Motor1_Speed(int Target_Speed);
void Set_Motor2_Speed(int Target_Speed);
void Turn_Right(float angle);
void Turn_Left(float angle);

#endif
