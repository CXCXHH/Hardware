#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "PID.h"

// 外部引用的PID控制器实例
extern PID_TypeDef pid_motor1;
extern PID_TypeDef pid_motor2;
extern PID_TypeDef pid_position;
extern PID_TypeDef pid_tuen;
extern PID_TypeDef pid_angle;

// 外部引用的全局目标值
extern float Target_Speed;
extern float Target_Position;

typedef enum
{
    TASK_NONE,
    TASK_SHORT,
    TASK_MIDDLE,
    TASK_LONG,
} TaskType_t;


typedef enum 
{
    //空闲
    STATE_IDLE,                     // 空闲状态

    //通用流程 
    STATE_INITIATE_FORWARD,         //开始第一段前进
    STATE_WAIT_FOR_FORWARD,         //等待第一段前进完成
    STATE_EXECUTE_TURN,             //执行转向 (在此状态下根据条件决定任务)
    STATE_INITIATE_FINAL_MOVE,      //开始第二段前进
    STATE_WAIT_FOR_FINAL_MOVE,      //等待第二段前进完成
    STATE_LONG_JUDGE,                //长距离判断 (在此状态下根据条件决定任务)
    STATE_LONG_TRUE,                 //长距离判断成功
    STATE_LONG_FALSE,                //长距离判断失败
    STATE_LONG_RECALL,               //长距离返回
    STATE_LONG_RECALL_TURN,         //长距离返回中的转向
    STATE_LONG_RECALL_DISTANCE,      //长距离返回中的距离

    //停止与等待返回
    STATE_STOP,                     //电机停止，等待红外信号

    //返回流程
    STATE_RECALL_EXECUTE_TURN_180,      //执行180度掉头 
    STATE_RECALL_INITIATE_MOVE_BACK,    //开始返回途中的第一次前进
    STATE_RECALL_WAIT_FOR_MOVE_BACK,    //等待第一次前进完成
    STATE_RECALL_EXECUTE_FINAL_TURN,    //执行返回途中的第二次转向
    STATE_RECALL_LONG_2,                //长距离回溯2 
    STATE_WAIT_FOR_RECALL_LONG_2,        //等待长距离回溯2完成
    STATE_RECALL_LONG_TURN_2,           //长距离回溯2中的转向
    STATE_WAIT_FOR_RECALL_LONG_TURN_2,        //等待长距离回溯2中的转向完成
    STATE_RECALL_INITIATE_FINAL_MOVE,   //开始返回途中的最后一次前进
    STATE_RECALL_WAIT_FOR_FINAL_MOVE,   //等待最后一次前进完成

} ControlState_t;


void Control_Init(void);
void Control(void);

void Control_Set_State(ControlState_t new_state);


#endif