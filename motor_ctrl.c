#include "motor_ctrl.h"
#include "jy61p.h"
#include "gw_gray.h"
#include "PID.h"
#include "Control.h"
#include "Encoder.h"

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

void Turn_Right(float angle)
{
  float target_angle, current_angle, error;
  float turn_angular_speed;
  int pwm_output1, pwm_output2;

  current_angle = get_angle()->z; // 获取当前Yaw角度
  target_angle = current_angle - angle; // 原地右转，Yaw角度减小

  //处理角度跳变
    if (target_angle < -180.0f)
    {
        target_angle += 360.0f;
    }

    pid_angle.integral = 0;

    do
    {
      current_angle = get_angle()->z; // 获取当前Yaw角度
      error = target_angle - current_angle; // 计算误差
      if (error > 180.0f) 
      {
          error -= 360.0f;
      } 
      else if (error < -180.0f) 
      {
          error += 360.0f;
      }
      turn_angular_speed = PID_realize(&pid_angle, 0, error);

      if(turn_angular_speed > MAX_TURN_ANGULAR_SPEED) turn_angular_speed = MAX_TURN_ANGULAR_SPEED;
      if(turn_angular_speed < -MAX_TURN_ANGULAR_SPEED) turn_angular_speed = -MAX_TURN_ANGULAR_SPEED;
      pwm_output1 = (int)PID_realize(&pid_motor1, turn_angular_speed * 10, Motor1_Speed);
      pwm_output2 = (int)PID_realize(&pid_motor2, -turn_angular_speed * 10, Motor2_Speed);

      Set_Motor1_Speed(pwm_output1);
      Set_Motor2_Speed(pwm_output2);
    }while (fabs(error) > ANGLE_ERROR_THRESHOLD);

    Motor1_Stop();
    Motor2_Stop();
}

void Turn_Left(float angle)
{
  float target_angle, current_angle, error;
  float turn_angular_speed;
  int pwm_output1, pwm_output2;

  current_angle = get_angle()->z; // 获取当前Yaw角度
  target_angle = current_angle + angle; // 原地右转，Yaw角度增加

  //处理角度跳变
    if (target_angle > 180.0f)
    {
        target_angle -= 360.0f;
    }

    pid_angle.integral = 0;

    do
    {
      current_angle = get_angle()->z; // 获取当前Yaw角度
      error = target_angle - current_angle; // 计算误差
      if (error > 180.0f) 
      {
          error -= 360.0f;
      } 
      else if (error < -180.0f) 
      {
          error += 360.0f;
      }
      turn_angular_speed = PID_realize(&pid_angle, 0, -error);

      if(turn_angular_speed > MAX_TURN_ANGULAR_SPEED) turn_angular_speed = MAX_TURN_ANGULAR_SPEED;
      if(turn_angular_speed < -MAX_TURN_ANGULAR_SPEED) turn_angular_speed = -MAX_TURN_ANGULAR_SPEED;
      pwm_output1 = (int)PID_realize(&pid_motor1, -turn_angular_speed * 10, Motor1_Speed);
      pwm_output2 = (int)PID_realize(&pid_motor2, turn_angular_speed * 10, Motor2_Speed);

      Set_Motor1_Speed(pwm_output1);
      Set_Motor2_Speed(pwm_output2);
    }while (fabs(error) > ANGLE_ERROR_THRESHOLD);

    Motor1_Stop();
    Motor2_Stop();
}

void Motor_Stop(void)
{
    Motor1_Stop();
    Motor2_Stop();
}