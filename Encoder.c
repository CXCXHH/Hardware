#include "Encoder.h"
#include "key.h"
#include "motor_ctrl.h"
#include "Control.h"

float Motor1_Speed = 0.0;
float Motor2_Speed = 0.0;

volatile int32_t Encoder_AB_Counter = 0;
volatile int32_t Encoder_CD_Counter = 0;


void GROUP1_IRQHandler(void)
{
    //获取A和C引脚的中断状态
    uint32_t triggered_interrupts = DL_GPIO_getEnabledInterruptStatus(Encoder_PORT, Encoder_A_PIN | Encoder_C_PIN);
    //判断是否是A引脚触发的中断
    if (triggered_interrupts & Encoder_A_PIN)
    {
        if(Read_Encoder_B)
        {
            Encoder_AB_Counter--;
        }
        else
        {
            Encoder_AB_Counter++;
        }
        DL_GPIO_clearInterruptStatus(Encoder_PORT, Encoder_A_PIN);
    }
    
    // 判断是否是C引脚触发的中断
    if (triggered_interrupts & Encoder_C_PIN)
    {
        if(Read_Encoder_D)
        {
            Encoder_CD_Counter--;
        }
        else
        {
            Encoder_CD_Counter++;
        }
        DL_GPIO_clearInterruptStatus(Encoder_PORT, Encoder_C_PIN);
    }
}

void Encoder_Speed(void)
{
    Motor1_Speed = 0.0;
    Motor2_Speed = 0.0;
    
    //上一次计数值
    static int32_t last_AB_Counter = 0;
    static int32_t last_CD_Counter = 0;

    //差值
    int32_t delta_AB_Counter;
    int32_t delta_CD_Counter;

    delta_AB_Counter = Encoder_AB_Counter - last_AB_Counter;
    last_AB_Counter = Encoder_AB_Counter; // 更新上一次计数值
    Motor1_Speed = delta_AB_Counter * DISTANCE_COUNT / SPEED_CALC_INTERVAL_S; // 左轮速度
    
    delta_CD_Counter = Encoder_CD_Counter - last_CD_Counter;
    last_CD_Counter = Encoder_CD_Counter; // 更新上一次计数值
    Motor2_Speed = -delta_CD_Counter * DISTANCE_COUNT / SPEED_CALC_INTERVAL_S; // 右轮速度
}

float Encoder_AB_Distance(void)
{
    return Encoder_AB_Counter * DISTANCE_COUNT; // 返回左轮行驶距离
}

float Encoder_CD_Distance(void)
{
    return -Encoder_CD_Counter * DISTANCE_COUNT; // 返回右轮行驶距离
}

void Encoder_Reset_Distance(void)
{
    Encoder_AB_Counter = 0;
    Encoder_CD_Counter = 0;
}

/**
 * @brief 使用PID速度环行驶指定距离
 * @param distance_cm  要行驶的目标距离 (单位: cm)
 * @param speed_cm_s   行驶时要保持的目标速度 (单位: cm/s)
 */
void Move_With_PID(float distance_cm, float speed_cm_s)
{
    float current_distance = 0.0f;

    // 1. 重置编码器里程计，从零开始计算距离
    Encoder_Reset_Distance(); //

    // 2. 设定全局的目标速度，让定时器中断中的PID控制器开始工作
    Target_Speed = speed_cm_s; //

    // 3. 进入循环，直到行驶的平均距离达到目标
    //    循环期间，真正的电机PWM控制是由定时器中断中的PID自动完成的
    while(current_distance < distance_cm)
    {
        // 计算左轮和右轮的平均行驶距离
        current_distance = (Encoder_AB_Distance() + Encoder_CD_Distance()) / 2.0f; //
        
        // 此处可以加入一些延时或调试代码，例如通过串口发送current_distance
        // delay_ms(5); 
    }

    // 4. 到达目标距离后，将目标速度设置为0，让PID控制器平稳刹车
    Target_Speed = 0.0f;

    // 5. 为确保完全停止，可以额外调用一次电机停止函数（可选，但推荐）
    Motor1_Stop(); //
    Motor2_Stop(); //
}