#include "motor_ctrl.h"

//PWM限制
void PWM_Limit(int *a, int ABS_MAX)
{
  if (*a > ABS_MAX)
    *a = ABS_MAX;
  if (*a < -ABS_MAX)
    *a = -ABS_MAX;
}
//设置电机1 PWM
void Set_Motor1_PWm(int Target_PWM)
{
    PWM_Limit(&Target_PWM, 9999);
    DL_TimerA_setCaptureCompareValue(PWM_0_INST,Target_PWM,GPIO_PWM_0_C1_IDX);
}
//设置电机2 PWM
void Set_Motor2_PWm(int Target_PWM)
{
    PWM_Limit(&Target_PWM, 9999);
    DL_TimerA_setCaptureCompareValue(PWM_0_INST, Target_PWM, GPIO_PWM_0_C0_IDX);
}

void Set_Motor1_Speed(int Target_Speed)
{
  if (Target_Speed > 0)
  {
    Motor1_Forward();
    Set_Motor1_PWm(Target_Speed);
  }
  else if (Target_Speed < 0)
  {
    Motor1_Backward();
    Set_Motor1_PWm(-Target_Speed);
  }
}

void Set_Motor2_Speed(int Target_Speed)
{
  if (Target_Speed > 0)
  {
    Motor2_Forward();
    Set_Motor2_PWm(Target_Speed);
  }
  else if (Target_Speed < 0)
  {
    Motor2_Backward();
    Set_Motor2_PWm(-Target_Speed);
  }
}