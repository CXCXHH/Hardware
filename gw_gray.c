#include "gw_gray.h"
#include "PID.h"
#include "Control.h"
#include "motor_ctrl.h"
#include "Encoder.h"
#include "jy61p.h"

PID_TypeDef pid_tuen; // 转向PID
PID_TypeDef pid_angle; // 角度PID
uint8_t count = 0; // 黑线计数器

const int8_t Sensor_Weights[] = {8, 6, 4, 2, -2, -4, -6, -8};// 传感器权重

uint8_t Read_Sensor(void)
{
    int black_count = 0;
    uint8_t sensor_value = 0;
    if(DL_GPIO_readPins(Huidu_L_1_PORT, Huidu_L_1_PIN)) sensor_value |= (1 << 0);
    if(DL_GPIO_readPins(Huidu_L_2_PORT, Huidu_L_2_PIN)) sensor_value |= (1 << 1);
    if(DL_GPIO_readPins(Huidu_L_3_PORT, Huidu_L_3_PIN)) sensor_value |= (1 << 2);
    if(DL_GPIO_readPins(Huidu_M_1_PORT, Huidu_M_1_PIN)) sensor_value |= (1 << 3);
    if(DL_GPIO_readPins(Huidu_M_2_PORT, Huidu_M_2_PIN)) sensor_value |= (1 << 4);
    if(DL_GPIO_readPins(Huidu_R_1_PORT, Huidu_R_1_PIN)) sensor_value |= (1 << 5);
    if(DL_GPIO_readPins(Huidu_R_2_PORT, Huidu_R_2_PIN)) sensor_value |= (1 << 6);
    if(DL_GPIO_readPins(Huidu_R_3_PORT, Huidu_R_3_PIN)) sensor_value |= (1 << 7);
    
    for(int i = 0; i < 8; i++)
    {
        if( ((sensor_value >> i) & 1) )
        {
            black_count++;
        }
    }

    if(black_count >= 6)
    {
        count=1;
    }

    return sensor_value;
}

void Line_Following(void)
{
    uint8_t sensor_value = Read_Sensor();
    static int last_error = 0; // 上一次误差
    float error = 0; // 当前误差
    int black_count = 0; // 黑线传感器计数
    float weighted_sum = 0; // 权重和

    for(int i = 0; i < 8; i++)
    {
        if( !((sensor_value >> i) & 1) )
        {
            weighted_sum += Sensor_Weights[i]; // 累加权重
            black_count++;
        }
    }
    if(black_count > 0)
    {
        error = weighted_sum / black_count; // 计算平均误差
        last_error = error; // 更新上一次误差
    }
    else
    {
        Motor1_Stop();
        Motor2_Stop();     
    }

    // 根据误差计算转向控制量
    float turn_output = PID_realize(&pid_tuen, 0, error);

    // 将转向控制应用到速度目标值
    float motor1_target_speed = LINE_FOLLOW_BASE_SPEED - turn_output;
    float motor2_target_speed = LINE_FOLLOW_BASE_SPEED + turn_output;

    //内环：速度PI控制器
	static float filtered_speed1 = 0.0f;
	static float filtered_speed2 = 0.0f;
	const float alpha = 0.80f; 
	filtered_speed1 = alpha * Motor1_Speed + (1.0f - alpha) * filtered_speed1;
	filtered_speed2 = alpha * Motor2_Speed + (1.0f - alpha) * filtered_speed2;


	int pwm_output1 = (int)PID_realize(&pid_motor1, motor1_target_speed*10, filtered_speed1);
    int pwm_output2 = (int)PID_realize(&pid_motor2, motor2_target_speed*10, filtered_speed2);


    Set_Motor1_Speed(pwm_output1);
    Set_Motor2_Speed(pwm_output2);
}

void PID_velocity_Position_and_Line_Following(float expected_distance)
{
    // 使用期望距离更新目标位置
    Target_Position = expected_distance;

    // 位置环控制
    float current_position = (Encoder_AB_Distance() + Encoder_CD_Distance()) / 2.0f;
    float desired_speed = PID_realize(&pid_position, Target_Position, current_position);

    // 对位置环输出的速度进行限幅
    if (desired_speed > Target_Speed)
    {
        desired_speed = Target_Speed;
    }
    else if (desired_speed < -Target_Speed)
    {
        desired_speed = -Target_Speed;
    }

    // 转向环控制
    uint8_t sensor_value = Read_Sensor();
    int black_count = 0;
    float weighted_sum = 0;

    for(int i = 0; i < 8; i++)
    {
        if( !((sensor_value >> i) & 1) )
        {
            weighted_sum += Sensor_Weights[i];
            black_count++;
        }
    }

    if(black_count > 0)
    {
        float error = weighted_sum / black_count;
        float turn_output = PID_realize(&pid_tuen, 0, error);

        // 将转向控制应用到速度目标值
        float motor1_target_speed = desired_speed - turn_output;
        float motor2_target_speed = desired_speed + turn_output;

        // 内环：速度PI控制器
        static float filtered_speed1 = 0.0f;
	    static float filtered_speed2 = 0.0f;
        const float alpha = 0.80f; 
	    filtered_speed1 = alpha * Motor1_Speed + (1.0f - alpha) * filtered_speed1;
	    filtered_speed2 = alpha * Motor2_Speed + (1.0f - alpha) * filtered_speed2;

        // 设置电机速度
        int pwm_output1 = (int)PID_realize(&pid_motor1, motor1_target_speed*10, filtered_speed1);
        int pwm_output2 = (int)PID_realize(&pid_motor2, motor2_target_speed*10, filtered_speed2);

        Set_Motor1_Speed(pwm_output1);
        Set_Motor2_Speed(pwm_output2);
    }
    else
    {
        Motor1_Stop();
        Motor2_Stop();
    }
}