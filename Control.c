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
// 0: 表示任务去程是右转，所以返程时需要左转
// 1: 表示任务去程是左转，所以返程时需要右转
static uint8_t g_recall_turn_direction = 0; 

// 0: 表示任务去程是右转，所以返程时需要左转
// 1: 表示任务去程是左转，所以返程时需要右转
static uint8_t Long_recall_turn_direction = 0;

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
    if(g_current_state != new_state)
    {
        g_current_state = new_state;
        // 在每次开始新的直线运动前，重置编码器累计的距离
        if (new_state == STATE_INITIATE_FORWARD   || 
            new_state == STATE_INITIATE_FINAL_MOVE ||
            new_state == STATE_RECALL_INITIATE_MOVE_BACK   ||
            new_state == STATE_RECALL_INITIATE_FINAL_MOVE  ||
            new_state == STATE_LONG_TRUE)
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

// 提取目标距离计算函数
float get_initial_distance(uint8_t detected_number, float current_dist)
{
    if (detected_number == 1 || detected_number == 2) {
        return 82.75;
    }
    else if (current_dist >= 190.0f) {
        return 260;
    }
    else {
        return 175.0;
    }
}

/**
 * @brief 主控制函数
 */
void Control(void)
{
    uint8_t star_signal = (DL_GPIO_readPins(Huidu_honwai_PORT, Huidu_honwai_PIN) == 0);
    uint8_t detected_number = Get_MaixCAM_Expected_Number();

    //只要不在空闲状态，并且有起始信号，就启动任务
    if (g_current_state == STATE_IDLE && star_signal && detected_number != 0)
    {
         Control_Set_State(STATE_INITIATE_FORWARD);
    }

    //主状态机
    switch (g_current_state)
    {
        case STATE_IDLE:
            Motor_Stop();
            break;

        //通用流程
        case STATE_INITIATE_FORWARD:
            {
                float current_dist = (Encoder_AB_Distance() + Encoder_CD_Distance()) / 2.0f;
                float initial_distance = get_initial_distance(detected_number, current_dist);
                
                PID_velocity_Position_and_Line_Following(initial_distance); 
                Control_Set_State(STATE_WAIT_FOR_FORWARD);
            }
            break;

        case STATE_WAIT_FOR_FORWARD:
            {
                // 使用上一步的目标距离，不再重新计算
                float current_dist = (Encoder_AB_Distance() + Encoder_CD_Distance()) / 2.0f;
                float initial_distance = get_initial_distance(detected_number, current_dist);
                
                PID_velocity_Position_and_Line_Following(initial_distance); 
                
                if (is_distance_reached()) 
                {
                    Control_Set_State(STATE_EXECUTE_TURN); // 到达后，进入转向决策状态
                }
            }
            break;

        case STATE_EXECUTE_TURN:
            {
                float current_dist = (Encoder_AB_Distance() + Encoder_CD_Distance()) / 2.0f;
                

                if (detected_number > 2 && Get_MaixCAM_Frame_Counter()>2) 
                {
                    // 条件满足，判定为中距离
                    g_current_task_type = TASK_MIDDLE;

                    if (current_dist >= 175.0f) 
                    {
                        uint8_t direction = Get_MaixCAM_Direction();
                        if (direction == 1) { // 1: 左转
                            Turn_Left(90);
                            g_recall_turn_direction = 1;
                            Control_Set_State(STATE_INITIATE_FINAL_MOVE);
                        } else if (direction == 2) { // 2: 右转
                            Turn_Right(90);
                            g_recall_turn_direction = 0;
                            Control_Set_State(STATE_INITIATE_FINAL_MOVE);
                        }
                    }
                }
                else if (detected_number >2 &&current_dist >260)
                {
                    // 条件满足，判定为远距离
                    g_current_task_type = TASK_LONG;

                    if (current_dist >= 260) 
                    {      
                        Turn_Left(90);
                    }
                    Control_Set_State(STATE_INITIATE_FINAL_MOVE);
                }
                else if(detected_number<=2)
                {
                    // 条件不满足，判定为短距离
                    g_current_task_type = TASK_SHORT;
                    if (detected_number == 2) { // 右转
                        Turn_Right(90);
                        g_recall_turn_direction = 0; 
                    } else { // 左转 
                        Turn_Left(90);
                        g_recall_turn_direction = 1; 
                    }
                    Control_Set_State(STATE_INITIATE_FINAL_MOVE);
                }
            }
            break;
        
        case STATE_INITIATE_FINAL_MOVE:
            {
                float final_distance;
                // 根据刚才判定的任务类型，决定第二段路程的距离
                switch (g_current_task_type)
                {
                case TASK_SHORT:
                case TASK_MIDDLE:
                    final_distance = 20.0;    
                    break;
                case TASK_LONG:
                    final_distance = 87.0;
                    break;
                default:
                    break;
                }
                PID_velocity_Position_and_Line_Following(final_distance);
                Control_Set_State(STATE_WAIT_FOR_FINAL_MOVE);
            }
            break;

        case STATE_WAIT_FOR_FINAL_MOVE:
            {
                float final_distance;
                switch (g_current_task_type)
                {
                case TASK_SHORT:
                case TASK_MIDDLE:
                    final_distance = 20.0;    
                    break;
                case TASK_LONG:
                    final_distance = 87.0;
                    break;
                default:
                    break;
                }
                PID_velocity_Position_and_Line_Following(final_distance);
                if(g_current_task_type==TASK_LONG)
                {
                    Control_Set_State(STATE_LONG_JUDGE);
                }
                else if (is_distance_reached()) 
                {
                    Control_Set_State(STATE_STOP);
                }
                
            }
            break;

        case STATE_LONG_JUDGE:
            {
                if (Get_MaixCAM_Frame_Counter() > 2) 
                {
                    uint8_t direction = Get_MaixCAM_Direction();
                        if (direction == 1) { // 1: 左转
                            Turn_Left(90);
                            g_recall_turn_direction = 1;
                            Control_Set_State(STATE_LONG_TRUE);
                        } else if (direction == 2) { // 2: 右转
                            Turn_Right(90);
                            g_recall_turn_direction = 0;
                            Control_Set_State(STATE_LONG_TRUE);
                        }
                }
                else
                {
                    Turn_Left(180);
                    Control_Set_State(STATE_LONG_FALSE);
                }
            }
            break;
        case STATE_LONG_TRUE:
            {
                Long_recall_turn_direction=1;
                PID_velocity_Position_and_Line_Following(40.0);
                if (is_distance_reached()) 
                {
                    Control_Set_State(STATE_STOP);
                }
            }
            break;
        case STATE_LONG_FALSE:
            {
                Long_recall_turn_direction=0;
                PID_velocity_Position_and_Line_Following(177.0);
                if (is_distance_reached()) 
                {
                 uint8_t direction = Get_MaixCAM_Direction();
                        if (direction == 1) { // 1: 左转
                            Turn_Left(90);
                            g_recall_turn_direction = 1;
                            Control_Set_State(STATE_STOP);
                        } else if (direction == 2) { // 2: 右转
                            Turn_Right(90);
                            g_recall_turn_direction = 0;
                            Control_Set_State(STATE_STOP);
                        }
                }
            }
            break;
        // --- 停止与回溯 ---
        case STATE_STOP:
            Motor_Stop();
            if (DL_GPIO_readPins(Huidu_honwai_PORT, Huidu_honwai_PIN) != 0)
            {
                Control_Set_State(STATE_RECALL_EXECUTE_TURN_180); 
            }
            break;
        
        case STATE_RECALL_EXECUTE_TURN_180:
             if (g_recall_turn_direction == 0) {
                Turn_Left(180);
            } else { 
                Turn_Right(180);               
            }
            Control_Set_State(STATE_RECALL_INITIATE_MOVE_BACK);
            break;

        case STATE_RECALL_INITIATE_MOVE_BACK:
            {
                float recall_dist_1 = 27.5;
                PID_velocity_Position_and_Line_Following(recall_dist_1);
                Control_Set_State(STATE_RECALL_WAIT_FOR_MOVE_BACK);
            }
            break;

        case STATE_RECALL_WAIT_FOR_MOVE_BACK:
            {
                 float recall_dist_1 = 27.5;
                 PID_velocity_Position_and_Line_Following(recall_dist_1);
                 if(is_distance_reached()){
                     Control_Set_State(STATE_RECALL_EXECUTE_FINAL_TURN);
                 }
            }
            break;

        case STATE_RECALL_EXECUTE_FINAL_TURN:
            if (g_recall_turn_direction == 0) {
                Turn_Left(70);
            } else {
                Turn_Right(90);
            }
            Control_Set_State(STATE_RECALL_INITIATE_FINAL_MOVE);
            break;
            
        case STATE_RECALL_INITIATE_FINAL_MOVE:
            {
                float recall_dist_2;
                switch (g_current_task_type) 
                {
                    case TASK_SHORT:
                        recall_dist_2 = 60.0;
                        break;
                    case TASK_MIDDLE:
                        recall_dist_2 = 145.0; 
                        break;
                    case TASK_LONG:
                        Control_Set_State(STATE_LONG_RECALL);
                        break;
                    default:
                        break;
                }
                PID_velocity_Position_and_Line_Following(recall_dist_2);
                Control_Set_State(STATE_RECALL_WAIT_FOR_FINAL_MOVE);
            }
            break;

        case STATE_RECALL_WAIT_FOR_FINAL_MOVE:
            {
                float recall_dist_2;
                switch (g_current_task_type) 
                {
                    case TASK_SHORT:
                        recall_dist_2 = 60.0;
                        break;
                    case TASK_MIDDLE:
                        recall_dist_2 = 145.0; 
                        break;
                    default:
                        break;
                }
                PID_velocity_Position_and_Line_Following(recall_dist_2);
                if (is_distance_reached()) {
                    g_current_task_type = TASK_NONE; // 清除任务记录
                    Control_Set_State(STATE_IDLE); 
                }
            }
            break;
        case STATE_LONG_RECALL:
            {
                PID_velocity_Position_and_Line_Following(90.0);
                if (is_distance_reached()) 
                {
                    Control_Set_State(STATE_LONG_RECALL_TURN);
                }
            }
            break;
        case STATE_LONG_RECALL_TURN:
            {
                if (Long_recall_turn_direction==0)
                {
                    Turn_Left(80);
                }
                else
                {
                    Turn_Right(80);
                }
                Control_Set_State(STATE_LONG_RECALL_DISTANCE);
            }
            break;
        case STATE_LONG_RECALL_DISTANCE:
            {
                PID_velocity_Position_and_Line_Following(257.5);
                if (is_distance_reached()) {
                    g_current_task_type = TASK_NONE; // 清除任务记录
                    Control_Set_State(STATE_IDLE); 
                }
            }
            break;

        default:
            Control_Set_State(STATE_IDLE);
            break;
    }
}