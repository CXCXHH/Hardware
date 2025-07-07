#ifndef __PID_H
#define __PID_H

#include "main.h"

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

#endif