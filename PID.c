#include "PID.h"
#include "Encoder.h"
#include "Control.h"
#include "motor_ctrl.h"

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

//位置式PID
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

void PID_velocity_Position(void)
{
    //外环：位置PID控制器 
    float current_position = (Encoder_AB_Distance() + Encoder_CD_Distance()) / 2.0f;

    //位置环PD计算：
    float desired_speed = PID_realize(&pid_position, Target_Position, current_position);

    // 3. 对位置环输出的速度进行限幅
    if (desired_speed > POSITION_MAX_SPEED_CM_S) 
    {
        desired_speed = POSITION_MAX_SPEED_CM_S;
    } 
    else if (desired_speed < -POSITION_MAX_SPEED_CM_S) 
    {
        desired_speed = -POSITION_MAX_SPEED_CM_S;
    }
    
    Target_Speed = desired_speed;

    //内环：速度PI控制器
	static float filtered_speed1 = 0.0f;
	static float filtered_speed2 = 0.0f;
	const float alpha = 0.80f; 
	filtered_speed1 = alpha * Motor1_Speed + (1.0f - alpha) * filtered_speed1;
	filtered_speed2 = alpha * Motor2_Speed + (1.0f - alpha) * filtered_speed2;


	int pwm_output1 = (int)PID_realize(&pid_motor1, Target_Speed*10, filtered_speed1);
    int pwm_output2 = (int)PID_realize(&pid_motor2, Target_Speed*10, filtered_speed2);


    Set_Motor1_Speed(pwm_output1);
    Set_Motor2_Speed(pwm_output2);
}


