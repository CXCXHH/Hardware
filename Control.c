#include "Control.h"
#include "motor_ctrl.h"
#include "PID.h"
#include "MaixCAM.h"
#include "gw_gray.h"
#include "Encoder.h"
#include <math.h> 
#include "jy61p.h"

// 全局静态变量，用于存储当前控制状态
static ControlState_t g_current_state = STATE_IDLE;
// 全局静态变量，用于记录当前执行的任务类型
static TaskType_t g_current_task_type = TASK_NONE;
// 全局静态变量，用于记录返回时应该朝哪个方向转弯
static uint8_t g_recall_turn_direction = 0; // 0: 去程右转，返程左转; 1: 去程左转，返程右转
static uint8_t Long_recall_turn_direction = 0;//0:左转，1：右转
static uint8_t Long_Flag = 0;

// 任务参数定义（可自行修改）
#define NEAR_DISTANCE         82.75  // 近端第一段距离
#define NEAR_FINAL_DISTANCE   20.0   // 近端第二段距离
#define MID_DISTANCE          175.0  // 中端第一段距离
#define MID_FINAL_DISTANCE    20.0   // 中端第二段距离
#define FAR_DISTANCE          265.0  // 远端第一段距离
#define FAR_TURN_DISTANCE     80.0   // 远端转弯后距离
#define FAR_RETURN_DISTANCE   170.0  // 远端无方向时的返回距离
#define FAR_TURN_DISTANCE_1   75.0   // 远端转弯后距离
#define FAR_FINAL_DISTANCE    25.0   // 远端最终距离

// 为了代码清晰和方便移植，建议将速度值也定义为宏
#define SPEED_HIGH 100.0f // 假设高速为100
#define SPEED_LOW  50.0f  // 假设低速为50

/**
 * @brief 初始化控制模块
 */
void Control_Init(void) 
{
    Control_Set_State(STATE_IDLE);
    g_current_task_type = TASK_NONE;
}

/**
 * @brief 设置新的控制状态，并在需要时重置传感器数据
 * @param new_state 新的状态
 */
void Control_Set_State(ControlState_t new_state)
{
    if (g_current_state != new_state)
    {
        g_current_state = new_state;
        // 在每次开始新的直线运动前，重置编码器累计的距离
        if (new_state == STATE_INITIATE_FORWARD   || 
            new_state == STATE_INITIATE_FINAL_MOVE ||
            new_state == STATE_RECALL_INITIATE_MOVE_BACK   ||
            new_state == STATE_RECALL_INITIATE_FINAL_MOVE  ||
            new_state == STATE_LONG_TRUE ||
            new_state == STATE_LONG_FALSE ||
            new_state == STATE_RECALL_LONG_2
            )
        {
            Encoder_Reset_Distance();
        }
    }
}

/**
 * @brief 检查小车是否走到了预设的目标距离
 * @return 1: 已到达, 0: 未到达
 */
uint8_t is_distance_reached(void) 
{
    float current_position = (Encoder_AB_Distance() + Encoder_CD_Distance()) / 2.0f;
    if (fabs(Target_Position - current_position) < 2.0f) 
    { 
        return 1;
    }
    return 0;
}

uint8_t is_turn_completed(void)
{
    Gyro_Struct *gyro_data = get_angle();
    if (fabs(pid_angle.target - gyro_data->z) < 1.5f)
    {
        jy61pInit();
        return 1;
    }
    return 0;
}

/**
 * @brief 主控制函数
 */
void Control(void)
{
    uint8_t start_signal = (DL_GPIO_readPins(Huidu_honwai_PORT, Huidu_honwai_PIN) == 0);
    uint8_t detected_number = Get_MaixCAM_Expected_Number();

    // 启动任务：不在空闲状态且有起始信号
    if (g_current_state == STATE_IDLE && start_signal && detected_number != 0)
    {
        Control_Set_State(STATE_INITIATE_FORWARD);
    }

    // 主状态机
    switch (g_current_state)
    {
        case STATE_IDLE:
            Motor_Stop();
            break;

        // 第一段前进
        case STATE_INITIATE_FORWARD:
        {
            // 这里可以设置初始前进速度
            Target_Speed = SPEED_HIGH; 
            float initial_distance = (detected_number == 1 || detected_number == 2) ? NEAR_DISTANCE : FAR_DISTANCE;
            PID_velocity_Position_and_Line_Following(initial_distance); 
            Control_Set_State(STATE_WAIT_FOR_FORWARD);
            break;
        }

        case STATE_WAIT_FOR_FORWARD:
        {
            float current_dist = (Encoder_AB_Distance() + Encoder_CD_Distance()) / 2.0f;
            if (detected_number == 1 || detected_number == 2)
            {
                g_current_task_type = TASK_SHORT;
                PID_velocity_Position_and_Line_Following(NEAR_DISTANCE);
            }
            else if (current_dist > 100.0f && Get_MaixCAM_Frame_Counter() > 2 &&current_dist<170)
            {
                g_current_task_type = TASK_MIDDLE;
                PID_velocity_Position_and_Line_Following(MID_DISTANCE);
            }
            else
            {
                g_current_task_type = TASK_LONG;
                PID_velocity_Position_and_Line_Following(FAR_DISTANCE);
            }

            if (is_distance_reached()) 
            {
                Control_Set_State(STATE_EXECUTE_TURN);
            }
            break;
        }

        case STATE_EXECUTE_TURN:
        {
            if (g_current_task_type == TASK_SHORT)
            {
                if (detected_number == 1) // 左转
                {
                    Turn_Left(90);
                    g_recall_turn_direction = 1;
                }
                else if (detected_number == 2) // 右转
                {
                    Turn_Right(90);
                    g_recall_turn_direction = 0;
                }
            }
            else if (g_current_task_type == TASK_MIDDLE)
            {
                uint8_t direction = Get_MaixCAM_Direction();
                if (direction == 1) // 左转
                {
                    Turn_Left(90);
                    g_recall_turn_direction = 1;
                }
                else if (direction == 2) // 右转
                {
                    Turn_Right(90);
                    g_recall_turn_direction = 0;
                }
            }
            else if (g_current_task_type == TASK_LONG)
            {
                Turn_Left(90); // 远端固定左转
                g_recall_turn_direction = 1;
            }
            Control_Set_State(STATE_INITIATE_FINAL_MOVE);
            break;
        }

        case STATE_INITIATE_FINAL_MOVE:
        {
            float final_distance = (g_current_task_type == TASK_SHORT) ? NEAR_FINAL_DISTANCE :
                                  (g_current_task_type == TASK_MIDDLE) ? MID_FINAL_DISTANCE : FAR_TURN_DISTANCE;
            PID_velocity_Position_and_Line_Following(final_distance);
            Control_Set_State(STATE_WAIT_FOR_FINAL_MOVE);
            break;
        }

        case STATE_WAIT_FOR_FINAL_MOVE:
        {
            // 在等待阶段，需要持续调用控制函数以维持运动
            float final_distance = (g_current_task_type == TASK_SHORT) ? NEAR_FINAL_DISTANCE :
                                  (g_current_task_type == TASK_MIDDLE) ? MID_FINAL_DISTANCE : FAR_TURN_DISTANCE;
            PID_velocity_Position_and_Line_Following(final_distance);

            if (is_distance_reached()) 
            {
                if (g_current_task_type == TASK_LONG)
                {
                    Control_Set_State(STATE_LONG_JUDGE);
                }
                else
                {
                    Control_Set_State(STATE_STOP);
                }
            }
            break;
        }

        case STATE_LONG_JUDGE:
        {
            Long_Flag=1;
            uint8_t direction = Get_MaixCAM_Direction();
            if (Get_MaixCAM_Frame_Counter()>=2) // 无方向，右掉头180度
            {
                Turn_Right(180);
                Long_recall_turn_direction = 0;
                Control_Set_State(STATE_LONG_FALSE);
            }
            else // 有方向，按指令转
            {
                Long_recall_turn_direction = 1;
                if (direction == 1) // 左转
                {
                    Turn_Left(90);
                    g_recall_turn_direction = 1;
                }
                else if (direction == 2) // 右转
                {
                    Turn_Right(90);
                    g_recall_turn_direction = 0;
                }
                Control_Set_State(STATE_LONG_TRUE);
            }
            break;
        }

        case STATE_LONG_TRUE:
        {
            PID_velocity_Position_and_Line_Following(FAR_FINAL_DISTANCE);
            if (is_distance_reached()) 
            {
                Control_Set_State(STATE_STOP);
            }
            break;
        }

        case STATE_LONG_FALSE:
        {
            
            PID_velocity_Position_and_Line_Following(FAR_RETURN_DISTANCE);
            if (is_distance_reached()) 
            {
                uint8_t direction = Get_MaixCAM_Direction();
                if (direction == 1) // 左转
                {
                    Turn_Left(90);
                    g_recall_turn_direction = 1;
                }
                else if (direction == 2) // 右转
                {
                    Turn_Right(90);
                    g_recall_turn_direction = 0;
                }
                Control_Set_State(STATE_LONG_TRUE);
            }
            break;
        }

        // 停止与回溯
        case STATE_STOP:
            Motor_Stop();
            if (DL_GPIO_readPins(Huidu_honwai_PORT, Huidu_honwai_PIN) != 0)
            {
                Control_Set_State(STATE_RECALL_EXECUTE_TURN_180); 
            }
            break;

        case STATE_RECALL_EXECUTE_TURN_180:
            if (g_recall_turn_direction == 0)
            {
                Turn_Left(180);
            }
            else
            {
                Turn_Right(180);               
            }
            Control_Set_State(STATE_RECALL_INITIATE_MOVE_BACK);
            break;

        case STATE_RECALL_INITIATE_MOVE_BACK:
        {

             
            float recall_dist_1 = 27.5; // 返回第一段距离
            PID_velocity_Position_and_Line_Following(recall_dist_1);
            Control_Set_State(STATE_RECALL_WAIT_FOR_MOVE_BACK);
            break;
        }

        case STATE_RECALL_WAIT_FOR_MOVE_BACK:
        {

            float recall_dist_1 = 27.5; 
            PID_velocity_Position_and_Line_Following(recall_dist_1);

            if (is_distance_reached())
            {
                Control_Set_State(STATE_RECALL_EXECUTE_FINAL_TURN);
            }
            break;
        }

        case STATE_RECALL_EXECUTE_FINAL_TURN:
            if (g_recall_turn_direction == 0)
            {
                Turn_Left(70); 
            }
            else
            {
                Turn_Right(90);
            }
            if(Long_Flag==1)
            {
                Control_Set_State(STATE_RECALL_LONG_2);
            }
            else
            {
                Control_Set_State(STATE_RECALL_INITIATE_FINAL_MOVE);
            }
            break;
        
        case STATE_RECALL_LONG_2:
        {
             
            float recall_dist_2 = FAR_TURN_DISTANCE_1; 
            PID_velocity_Position_and_Line_Following(recall_dist_2);
            Control_Set_State(STATE_WAIT_FOR_RECALL_LONG_2);
            break;
        }

        case STATE_WAIT_FOR_RECALL_LONG_2:
        {
            float recall_dist_2 = FAR_TURN_DISTANCE_1;
            PID_velocity_Position_and_Line_Following(recall_dist_2);

            if (is_distance_reached())
            {
                Control_Set_State(STATE_RECALL_LONG_TURN_2);
            }
            break;
        }      
        
        case STATE_RECALL_LONG_TURN_2:
        {
            if (Long_recall_turn_direction == 0)
            {
                Turn_Left(70); 
            }
            else
            {
                Turn_Right(90); 
            }
            Control_Set_State(STATE_RECALL_INITIATE_FINAL_MOVE);
            break;
        }



        case STATE_RECALL_INITIATE_FINAL_MOVE:
        {
 
            float recall_dist_2 = (g_current_task_type == TASK_SHORT) ? 60.0 :
                                 (g_current_task_type == TASK_MIDDLE) ? 145.0 : 247.5; 
            PID_velocity_Position_and_Line_Following(recall_dist_2);
            Control_Set_State(STATE_RECALL_WAIT_FOR_FINAL_MOVE);
            break;
        }

        case STATE_RECALL_WAIT_FOR_FINAL_MOVE:
        {
             Target_Speed = 80;
            float recall_dist_2 = (g_current_task_type == TASK_SHORT) ? 60.0 :
                                 (g_current_task_type == TASK_MIDDLE) ? 145.0 : 247.5;
            PID_velocity_Position_and_Line_Following(recall_dist_2);

            if (is_distance_reached())
            {
                Motor_Stop();
                g_current_task_type = TASK_NONE;
                Control_Set_State(STATE_IDLE); 
            }
            break;
        }

        default:
            Control_Set_State(STATE_IDLE);
            break;
    }
}
