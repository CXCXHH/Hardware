#include "Encoder.h"
#include "key.h"
#include "motor_ctrl.h"

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

void Move_Distance(float distance_cm, int pwm)
{
    float current_distance = 0.0f;
    Encoder_Reset_Distance();
    Set_Motor1_Speed(pwm);
    Set_Motor2_Speed(pwm);
    while(current_distance < distance_cm)
    {
        current_distance = (Encoder_AB_Distance()+ Encoder_CD_Distance()) / 2.0f;
    }

    Motor1_Stop();
    Motor2_Stop();
}