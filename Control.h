#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "PID.h"

extern PID_TypeDef pid_motor1;
extern PID_TypeDef pid_motor2;
extern PID_TypeDef pid_position;
extern float Target_Speed;
extern float Target_Position;

#endif
