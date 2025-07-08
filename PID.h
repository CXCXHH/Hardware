#ifndef __PID_H
#define __PID_H

#include "main.h"

#define POSITION_MAX_SPEED_CM_S  (50.0f) //位置环输出的最大速度

typedef struct
{
    float Kp; // 比例系数
    float Ki; // 积分系数
    float Kd; // 微分系数

    float target; // 目标值
    float actual; // 实际值

    float error; // 当前误差
    float last_error; // 上一次误差
    float integral; // 积分值

    float output; // 输出值
} PID_TypeDef;

void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd);
float PID_realize(PID_TypeDef *pid, float target, float actual);
void PID_velocity_Position(void);

#endif