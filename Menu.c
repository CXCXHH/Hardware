#include "Menu.h"
#include "oled.h"
#include "key.h"
#include "Encoder.h"
#include "jy61p.h"
#include "Control.h"
#include "PID.h"

//菜单状态结构体
typedef enum 
{
    MENU_STATE_MAIN,   //主菜单状态
    MENU_DATA_VIEW,   //数据查看状态
    MENU_PARAM_SET,  //参数设置状态
    MENU_SHOW_ENCODER, //显示编码器状态
    MENU_SHOW_GYRO,    //显示陀螺仪状态
    MENU_SHOW_VIEW,    //显示视图状态
    MENU_PID_TUNE,   //PID调参状态
    MENU_SET_SPEED, //设置速度
    MENU_PID_DEBUG,  //PID调试状态
} MenuState;



static uint8_t Menu_resquest = 1;// 菜单请求标志
static MenuState current_state = MENU_STATE_MAIN; // 当前菜单状态
static uint8_t Selected_index = 0; // 当前选中菜单项索引

static uint8_t tune_motor_select = 0; // 0: 电机1, 1: 电机2
static uint8_t tune_param_select = 0; // 0: Kp, 1: Ki, 2: Kd
static const float tune_step[] = {1.0f, 0.0f, 5.0f}; // Kp, Ki, Kd 调整步长 

//静态菜单显示
static void display_menu(void) 
{
   char buffer[32];
    OLED_Clear();

    switch (current_state)
    {
    case MENU_STATE_MAIN:
        OLED_ShowString(40, 0, (uint8_t *)"Menu", 16, 1);
        OLED_ShowString(10, 16, (uint8_t *)"Start", 16, 1);
        OLED_ShowString(10, 32, (uint8_t *)"Data View", 16, 1);
        OLED_ShowString(10, 48, (uint8_t *)"Param Set", 16, 1);
        OLED_ShowString(0, 16+Selected_index*16, (uint8_t *)">", 16, 1);
        break;

    case MENU_DATA_VIEW:
        OLED_ShowString(0, 0, (uint8_t *)"---Data View---", 16, 1);
        OLED_ShowString(10, 16, (uint8_t *)"Show Encoder", 16, 1);
        OLED_ShowString(10, 32, (uint8_t *)"Show Gyro", 16, 1);
        OLED_ShowString(10, 48, (uint8_t *)"Show view", 16, 1);
        OLED_ShowString(0, 16+Selected_index*16, (uint8_t *)">", 16, 1);
        break;
    case MENU_PARAM_SET:
        OLED_ShowString(0, 0, (uint8_t *)"---Param Set---", 16, 1);
        OLED_ShowString(10, 16, (uint8_t *)"PID Debug", 16, 1); 
        OLED_ShowString(10, 32, (uint8_t *)"Set Speed", 16, 1);
        OLED_ShowString(0, 16 + Selected_index * 16, (uint8_t *)">", 16, 1);
        break;
    case MENU_PID_DEBUG:
        OLED_ShowString(0, 0, (uint8_t *)"---PID Debug---", 16, 1);
        OLED_ShowString(10, 16, (uint8_t *)"Motor 1 PID", 16, 1);
        OLED_ShowString(10, 32, (uint8_t *)"Motor 2 PID", 16, 1);
        OLED_ShowString(10, 48, (uint8_t *)"Position PID", 16, 1);
        OLED_ShowString(0, 16 + Selected_index * 16, (uint8_t *)">", 16, 1);
        break;
    case MENU_SHOW_GYRO:
        OLED_ShowString(0, 0, (uint8_t *)"---Show Gyro---", 16, 1);
        break;
      case MENU_PID_TUNE:
        {
            char title[20];
            if (tune_motor_select < 2) 
            {
                snprintf(title, sizeof(title), "---Tune Motor%d---", tune_motor_select + 1);
            } 
            else 
            {
                snprintf(title, sizeof(title), "-Tune Position-");
            }
            OLED_ShowString(0, 0, (uint8_t *)title, 16, 1);
            OLED_ShowString(0, 16 + tune_param_select * 16, (uint8_t *)">", 16, 1);
        }
        break;
    case MENU_SET_SPEED: 
        OLED_ShowString(0, 0, (uint8_t *)"---Set Speed---", 16, 1);
        OLED_ShowString(0, 48, (uint8_t *)"K1+ K2- K4<Back", 16, 1); 
        break;
    }
    
}

static void Key_input(void)
{
    // 遍历所有按键
    for (int i = 0; i < KEY_Number; i++)
    {
        int key_id = i;
        // 检查是否有短按标志
        if (key[i].Short_Flag)
        {
            key[i].Short_Flag = 0; // 清除标志

            // 根据当前的菜单状态执行不同的操作
            switch (current_state)
            {
            case MENU_STATE_MAIN:
                Menu_resquest = 1; // 请求菜单刷新
                const int main_max_index = 2;
                if (key_id == 0) // KEY1: 上
                {
                    Selected_index = (Selected_index > 0) ? Selected_index - 1 : main_max_index;
                }
                else if (key_id == 1) // KEY2: 下
                {
                    Selected_index = (Selected_index < main_max_index) ? Selected_index + 1 : 0;
                }
                else if (key_id == 2) // KEY3: 确认
                {
                    switch (Selected_index)
                    {
                    case 0: // Start

                        break;
                    case 1: // Data View
                        current_state = MENU_DATA_VIEW; // 进入数据查看
                        Selected_index = 0;
                        break;
                    case 2: // Param Set
                        current_state = MENU_PARAM_SET; // 进入参数设置
                        Selected_index = 0;
                        break;
                    }
                }
                break;

            case MENU_DATA_VIEW:
                Menu_resquest = 1;
                const int data_view_max_index = 2;
                if (key_id == 0) // KEY1: 上
                {
                    Selected_index = (Selected_index > 0) ? Selected_index - 1 : data_view_max_index;
                }
                else if (key_id == 1) // KEY2: 下
                {
                    Selected_index = (Selected_index < data_view_max_index) ? Selected_index + 1 : 0;
                }
                else if (key_id == 2) // KEY3: 确认
                {
                    switch (Selected_index)
                    {
                    case 0: // Show Encoder
                        current_state = MENU_SHOW_ENCODER;
                        break;
                    case 1: // Show Gyro
                        current_state = MENU_SHOW_GYRO;
                        break;
                    case 2: // Show view
                        // 待实现
                        break;
                    }
                }
                else if (key_id == 3) // KEY4: 返回
                {
                    current_state = MENU_STATE_MAIN;
                    Selected_index = 0;
                }
                break;

              case MENU_PARAM_SET: 
                Menu_resquest = 1;
                const int param_set_max_index = 1;
                if (key_id == 0) // KEY1: 上
                {
                    Selected_index = (Selected_index > 0) ? Selected_index - 1 : param_set_max_index;
                }
                else if (key_id == 1) // KEY2: 下
                {
                    Selected_index = (Selected_index < param_set_max_index) ? Selected_index + 1 : 0;
                }
                else if (key_id == 2) // KEY3: 确认
                {
                    switch (Selected_index)
                    {
                    case 0: // PID Debug
                        current_state = MENU_PID_DEBUG;
                        break;
                    case 1: // Set Speed
                        current_state = MENU_SET_SPEED;
                        break;
                    case 2: 
                        
                        break;
                    }
                }
                else if (key_id == 3) // KEY4: 返回
                {
                    current_state = MENU_STATE_MAIN;
                    Selected_index = 0;
                }
                break;

            case MENU_PID_DEBUG:
                Menu_resquest = 1;
                const int pid_debug_max_index = 2;
                if (key_id == 0) // KEY1: 上
                {
                    Selected_index = (Selected_index > 0) ? Selected_index - 1 : pid_debug_max_index;
                }
                else if (key_id == 1) // KEY2: 下
                {
                    Selected_index = (Selected_index < pid_debug_max_index) ? Selected_index + 1 : 0;
                }
                else if (key_id == 2) // KEY3: 确认
                {
                    tune_motor_select = Selected_index; 
                    current_state = MENU_PID_TUNE; 
                    tune_param_select = 0;
                }
                else if (key_id == 3) // KEY4: 返回
                {
                    current_state = MENU_PARAM_SET; // 返回到 "Param Set"
                    Selected_index = 0; 
                }
                break;

            case MENU_PID_TUNE: 
                {
                    // 根据tune_motor_select选择要操作的PID
                    PID_TypeDef *p;
                    if (tune_motor_select == 0) 
                    {
                        p = &pid_motor1;
                    } 
                    else if (tune_motor_select == 1)
                    {
                        p = &pid_motor2;
                    } 
                    else if (tune_motor_select == 2) 
                    { 
                        p = &pid_position;
                    }

                    if (key_id == 0) // KEY1:上 
                    {
                        tune_param_select = (tune_param_select > 0) ? tune_param_select - 1 : 2; // 循环上移
                        Menu_resquest = 1; // 请求刷新以移动光标
                    }
                    else if (key_id == 1) // KEY2:下 
                    {
                        tune_param_select = (tune_param_select < 2) ? tune_param_select + 1 : 0; // 循环下移
                        Menu_resquest = 1; // 请求刷新以移动光标
                    }
                    else if (key_id == 2) // KEY3: 增加数值
                    {
                        if (tune_param_select == 0) p->Kp += tune_step[0];      // 增加Kp
                        else if (tune_param_select == 1) p->Ki += tune_step[1]; // 增加Ki
                        else if (tune_param_select == 2) p->Kd += tune_step[2]; // 增加Kd
                    }
                    else if (key_id == 3) // KEY4: 减少数值
                    {
                        if (tune_param_select == 0) p->Kp -= tune_step[0];      // 减少Kp
                        else if (tune_param_select == 1) p->Ki -= tune_step[1]; // 减少Ki
                        else if (tune_param_select == 2) p->Kd -= tune_step[2]; // 减少Kd                   
                    }
                }
                break;

            case MENU_SET_SPEED: 
                if (key_id == 0) // KEY1: 增加速度
                {
                    Target_Speed += 25.0f; // 步长为1.0
                }
                else if (key_id == 1) // KEY2: 减少速度
                {
                    Target_Speed -= 25.0f;
                    //if (Target_Speed < 0) Target_Speed = 0; // 防止速度为负
                }
                else if (key_id == 3) // KEY4: 返回
                {
                    current_state = MENU_PARAM_SET;
                    Selected_index = 0; 
                    Menu_resquest = 1;
                }
                break;
                
            case MENU_SHOW_ENCODER:
                if (key_id == 0) //KEY1
                {
                    Encoder_Reset_Distance(); // 重置编码器距离
                }
                else if (key_id == 3) //KEY4
                {
                    current_state = MENU_DATA_VIEW; // 返回数据查看状态
                    Selected_index = 0;             // 重置选中索引
                    Menu_resquest = 1;              // 菜单更新
                }
                break;

            case MENU_SHOW_GYRO:
                if (key_id == 3) //KEY4
                {
                    current_state = MENU_DATA_VIEW; // 返回数据查看状态
                    Selected_index = 0;             // 重置选中索引
                    Menu_resquest = 1;              // 菜单更新
                }
                break;
            }
        }

        if (key[key_id].Long_Flag)
        {
            key[key_id].Long_Flag = 0; 

            if (current_state == MENU_PID_TUNE && key_id == 3)
            {
                current_state = MENU_PARAM_SET; // 返回上一级菜单
                Selected_index = 0; 
                Menu_resquest = 1;
            }

            continue; 
        }
    }
}

static void Menu_Update(void) 
{
    char buffer[32];
    switch (current_state)
    {
    case MENU_SHOW_ENCODER:
        snprintf(buffer, sizeof(buffer), "L:%.1f cm/s", Motor1_Speed);
        OLED_ShowString(0, 0, (uint8_t *)buffer, 16, 1);
        snprintf(buffer, sizeof(buffer), "R:%.1f cm/s", Motor2_Speed);
        OLED_ShowString(0, 16, (uint8_t *)buffer, 16, 1); 
        snprintf(buffer, sizeof(buffer), "Dis_AB:%.2f cm", Encoder_AB_Distance());
        OLED_ShowString(0, 32, (uint8_t *)buffer, 16, 1);
        snprintf(buffer, sizeof(buffer), "Dis_CD:%.2f cm", Encoder_CD_Distance());
        OLED_ShowString(0, 48, (uint8_t *)buffer, 16, 1);
        break;
    
    case MENU_SHOW_GYRO:
        {
            Gyro_Struct *gyro_data = get_angle();
            snprintf(buffer, sizeof(buffer), "Roll : %-6.2f", gyro_data->x);
            OLED_ShowString(16, 16, (uint8_t *)buffer, 16, 1);
            snprintf(buffer, sizeof(buffer), "Pitch: %-6.2f", gyro_data->y);
            OLED_ShowString(16, 32, (uint8_t *)buffer, 16, 1);
            snprintf(buffer, sizeof(buffer), "Yaw: %-6.2f", gyro_data->z);
            OLED_ShowString(16, 48, (uint8_t *)buffer, 16, 1);
        }       
        break;
    case MENU_PID_TUNE:
        {
            PID_TypeDef *P;
            if (tune_motor_select == 0) 
            {
                P = &pid_motor1;
            } 
            else if (tune_motor_select == 1) 
            {
                P = &pid_motor2;
            } 
            else 
            { 
                P = &pid_position;                 
            }
            snprintf(buffer, sizeof(buffer), "P:%.3f", P->Kp);
            OLED_ShowString(16, 16, (uint8_t *)buffer, 16, 1);
            snprintf(buffer, sizeof(buffer), "I:%.3f", P->Ki);
            OLED_ShowString(16, 32, (uint8_t *)buffer, 16, 1);
            snprintf(buffer, sizeof(buffer), "D:%.3f", P->Kd);
            OLED_ShowString(16, 48, (uint8_t *)buffer, 16, 1);
        }      
        break;
    case MENU_SET_SPEED: 
        {
            snprintf(buffer, sizeof(buffer), "Target:%.1fcm/s", Target_Speed);
            OLED_ShowString(10, 24, (uint8_t *)buffer, 16, 1);
        }
        break;
    default:
        break;
    }
}

//菜单初始化
void Menu_Init(void) 
{
    OLED_Init();
}

//菜单循环更新
void Menu_loop(void) 
{
    Key_input();
    if(Menu_resquest)
    {
        display_menu();
        Menu_resquest = 0;
    }
    Menu_Update();
}
