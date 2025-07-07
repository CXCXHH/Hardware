#include "PID.h"

void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd)
{   
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->target = 0.0f;
    pid->actual = 0.0f;

    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;

    pid->output = 0.0f;
}

//增量式PID
float PID_realize(PID_TypeDef *pid, float target, float actual)
{
    pid->target = target;
    pid->actual = actual;

    pid->error = pid->target - pid->actual;

    pid->integral += pid->error;
    if(pid->integral > 3000.0f)
    {
        pid->integral = 3000.0f;
    }
    else if(pid->integral < -3000.0f)
    {
        pid->integral = -3000.0f;
    }
    pid->output = pid->Kp * pid->error + pid->Ki * pid->integral + pid->Kd * (pid->error - pid->last_error);

    pid->last_error = pid->error;

    return pid->output;
}