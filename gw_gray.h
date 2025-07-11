#ifndef __GW_GRAY_H__
#define __GW_GRAY_H__

#include "main.h" 

#define LINE_FOLLOW_BASE_SPEED  (Target_Speed)
extern PID_TypeDef pid_tuen;//转向PID
extern PID_TypeDef pid_angle; // 电机1 PID 



void Line_Following(void);
void PID_velocity_Position_and_Line_Following(float expected_distance);
#endif
