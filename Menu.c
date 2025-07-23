#include "Menu.h"
#include "oled.h"
#include "key.h"
#include "Encoder.h"
#include "jy61p.h"
#include "Control.h"
#include "PID.h"
#include "gw_gray.h" 
#include "motor_ctrl.h"
#include "MaixCAM.h"

//菜单状态结构体
typedef enum 
{
    MENU_STATE_MAIN,   //主菜单状态
    MENU_STATE_START,  //开始运行状态，用于显示视觉数据
    MENU_DATA_VIEW,   //数据查看状态
    MENU_SHOW_ENCODER, //显示编码器状态
    MENU_SHOW_GYRO,    //显示陀螺仪状态
    MENU_SHOW_VIEW,    //显示视图状态
    MENU_PARAM_SET,  //参数设置状态
    MENU_SET_SPEED, //设置速度
    MENU_SET_ANGLE,  //设置角度
    MENU_PID_DEBUG,  //PID调试状态
    MENU_PID_TUNE_MOTOR,   //PID调参状态
    MENU_PID_TUNE_TURN,   //转向环PID调试
    MENU_PID_TUNE_ANGLE,  //角度环PID调试
} MenuState;

static uint8_t Menu_resquest = 1;// 菜单请求标志
static MenuState current_state = MENU_STATE_MAIN; // 当前菜单状态
static uint8_t Selected_index = 0; // 当前选中菜单项索引
uint8_t g_line_following_enabled = 0; 
float g_target_angle_setting = 0.0f;
static uint8_t tune_motor_select = 0;
static uint8_t tune_param_select = 0;
static const float tune_step[] = {1.0f, 0.0f, 5.0f};

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
        OLED_ShowString(0, 16 + Selected_index * 16, (uint8_t *)">", 16, 1);     
        break;
    case MENU_STATE_START:
        OLED_ShowString(10, 0, (uint8_t *)"-- Running --", 16, 1);
        OLED_ShowString(0, 48, (uint8_t *)"K4<Stop", 16, 1);
        break;
    case MENU_DATA_VIEW:
        OLED_ShowString(0, 0, (uint8_t *)"---Data View---", 16, 1);
        OLED_ShowString(10, 16, (uint8_t *)"Show Encoder", 16, 1);
        OLED_ShowString(10, 32, (uint8_t *)"Show Gyro", 16, 1);
        OLED_ShowString(10, 48, (uint8_t *)"Show view", 16, 1);
        OLED_ShowString(0, 16 + Selected_index * 16, (uint8_t *)">", 16, 1);
        break;
    case MENU_PARAM_SET:
        OLED_ShowString(0, 0, (uint8_t *)"--Param Set--", 16, 1);
        OLED_ShowString(10, 16, (uint8_t *)"Set Speed", 16, 1);
        OLED_ShowString(10, 32, (uint8_t *)"Set Angle", 16, 1); 
        OLED_ShowString(10, 48, (uint8_t *)"PID Debug", 16, 1); 
        OLED_ShowString(0, 16 + Selected_index * 16, (uint8_t *)">", 16, 1);
        break;
    case MENU_PID_DEBUG:
        OLED_ShowString(0, 0, (uint8_t *)"---PID Debug---", 16, 1);
        OLED_ShowString(10, 16, (uint8_t *)"Motor PID", 16, 1);    
        OLED_ShowString(10, 32, (uint8_t *)"Turn PID", 16, 1);     
        OLED_ShowString(10, 48, (uint8_t *)"Angle PID", 16, 1);    
        OLED_ShowString(0, 16 + Selected_index * 16, (uint8_t *)">", 16, 1);
        break;
    case MENU_SHOW_ENCODER:
        OLED_ShowString(0, 0, (uint8_t *)"--Encoder Value--", 16, 1);
        OLED_ShowString(0, 48, (uint8_t *)"K1:Reset K4:Back", 12, 1);
        break;
    case MENU_SHOW_GYRO:
        OLED_ShowString(0, 0, (uint8_t *)"---Gyro Value---", 16, 1);
        OLED_ShowString(0, 48, (uint8_t *)"K4<Back", 16, 1);
        break;
    case MENU_SHOW_VIEW:
        OLED_ShowString(0, 0, (uint8_t *)"---GrayScale---", 16, 1);
        OLED_ShowString(0, 48, (uint8_t *)"K4<Back", 16, 1);
        break;
    case MENU_PID_TUNE_MOTOR:
        snprintf(buffer, sizeof(buffer), "-Tune Motor%d-", tune_motor_select + 1);
        OLED_ShowString(0, 0, (uint8_t *)buffer, 16, 1);
        OLED_ShowString(0, 16 + tune_param_select * 16, (uint8_t *)">", 16, 1);
        break;
    case MENU_SET_SPEED: 
        OLED_ShowString(0, 0, (uint8_t *)"---Set Speed---", 16, 1);
        OLED_ShowString(0, 48, (uint8_t *)"K1+ K2- K4<Back", 12, 1); 
        break;
    case MENU_SET_ANGLE:
        OLED_ShowString(0, 0, (uint8_t *)"--Set Angle--", 16, 1);
        OLED_ShowString(0, 48, (uint8_t *)"K1+ K2- K3:Test K4<Back", 12, 1); 
        break;
    case MENU_PID_TUNE_TURN:
        OLED_ShowString(0, 0, (uint8_t *)"-Tune Turn PID-", 16, 1);
        OLED_ShowString(0, 16 + tune_param_select * 16, (uint8_t *)">", 16, 1);
        break;
    case MENU_PID_TUNE_ANGLE:
        OLED_ShowString(0, 0, (uint8_t *)"-Tune Angle PID-", 16, 1);
        OLED_ShowString(0, 16 + tune_param_select * 16, (uint8_t *)">", 16, 1);
        break;
    default:
        break;
    }
}

static void Key_input(void)
{
    for (int i = 0; i < KEY_Number; i++)
    {
        int key_id = i;
        if (key[i].Short_Flag)
        {
            key[i].Short_Flag = 0;
            Menu_resquest = 1;
            

            switch (current_state)
            {
            case MENU_STATE_MAIN:
                {
                    const int max_index = 2;
                    if (key_id == 0) Selected_index = (Selected_index > 0) ? Selected_index - 1 : max_index;
                    else if (key_id == 1) Selected_index = (Selected_index < max_index) ? Selected_index + 1 : 0;
                    else if (key_id == 2) {
                        if (Selected_index == 0) {
                            current_state = MENU_STATE_START;
                            g_line_following_enabled = 1;
                        } else if (Selected_index == 1) {
                            current_state = MENU_DATA_VIEW;
                            Selected_index = 0;
                        } else if (Selected_index == 2) {
                            current_state = MENU_PARAM_SET;
                            Selected_index = 0;
                        }
                    }
                }
                break;
            case MENU_STATE_START:
                if (key_id == 3) {
                    current_state = MENU_STATE_MAIN;
                    g_line_following_enabled = 0;
                    Selected_index = 0;
                }
                break;
            case MENU_DATA_VIEW:
                {
                    const int max_index = 2;
                    if (key_id == 0) Selected_index = (Selected_index > 0) ? Selected_index - 1 : max_index;
                    else if (key_id == 1) Selected_index = (Selected_index < max_index) ? Selected_index + 1 : 0;
                    else if (key_id == 2) {
                        if(Selected_index == 0) current_state = MENU_SHOW_ENCODER;
                        else if(Selected_index == 1) current_state = MENU_SHOW_GYRO;
                        else if(Selected_index == 2) current_state = MENU_SHOW_VIEW;
                    } else if (key_id == 3) {
                        current_state = MENU_STATE_MAIN;
                        Selected_index = 1;
                    }
                }
                break;
            case MENU_PARAM_SET:
                {
                    const int max_index = 2;
                    if (key_id == 0) Selected_index = (Selected_index > 0) ? Selected_index - 1 : max_index;
                    else if (key_id == 1) Selected_index = (Selected_index < max_index) ? Selected_index + 1 : 0;
                    else if (key_id == 2) {
                        if(Selected_index == 0) current_state = MENU_SET_SPEED;
                        else if(Selected_index == 1) current_state = MENU_SET_ANGLE;
                        else if(Selected_index == 2) {
                            current_state = MENU_PID_DEBUG;
                            Selected_index = 0;
                        }
                    } else if (key_id == 3) {
                        current_state = MENU_STATE_MAIN;
                        Selected_index = 2;
                    }
                }
                break;
            case MENU_PID_DEBUG:
                {
                    const int max_index = 2;
                    if (key_id == 0) Selected_index = (Selected_index > 0) ? Selected_index - 1 : max_index;
                    else if (key_id == 1) Selected_index = (Selected_index < max_index) ? Selected_index + 1 : 0;
                    else if (key_id == 2) {
                        if(Selected_index == 0) current_state = MENU_PID_TUNE_MOTOR;
                        else if(Selected_index == 1) current_state = MENU_PID_TUNE_TURN;
                        else if(Selected_index == 2) current_state = MENU_PID_TUNE_ANGLE;
                        tune_param_select = 0;
                    } else if (key_id == 3) {
                        current_state = MENU_PARAM_SET;
                        Selected_index = 2;
                    }
                }
                break;
            case MENU_SHOW_ENCODER:
            case MENU_SHOW_GYRO:
            case MENU_SHOW_VIEW:
                if (key_id == 3) {
                    current_state = MENU_DATA_VIEW;
                    Selected_index = 0;
                }
                if (current_state == MENU_SHOW_ENCODER && key_id == 0) {
                    Encoder_Reset_Distance();
                }
                break;
            case MENU_SET_SPEED:
                if (key_id == 0) Target_Speed += 25.0f;
                else if (key_id == 1) Target_Speed -= 25.0f;
                else if (key_id == 3) {
                    current_state = MENU_PARAM_SET;
                    Selected_index = 0;
                }
                break;
            case MENU_SET_ANGLE:
                if (key_id == 0) g_target_angle_setting += 90.0f;
                else if (key_id == 1) g_target_angle_setting -= 90.0f;
                else if (key_id == 2) {
                    if(g_target_angle_setting > 0) Turn_Left(g_target_angle_setting);
                    else Turn_Right(-g_target_angle_setting);
                }
                else if (key_id == 3) {
                    current_state = MENU_PARAM_SET;
                    Selected_index = 1;
                }
                break;
            case MENU_PID_TUNE_MOTOR:
            case MENU_PID_TUNE_TURN:
            case MENU_PID_TUNE_ANGLE:
                {
                    PID_TypeDef *p = NULL;
                    // 根据当前状态选择要操作的PID
                    if (current_state == MENU_PID_TUNE_MOTOR) {
                        
                        p = (tune_motor_select == 0) ? &pid_motor1 : ((tune_motor_select == 1) ? &pid_motor2 : &pid_position);
                    } else if (current_state == MENU_PID_TUNE_TURN) {
                        p = &pid_tuen;
                    } else if (current_state == MENU_PID_TUNE_ANGLE) {
                        p = &pid_angle;
                    }

                    if (key_id == 0) // KEY1:上 
                    {
                        tune_param_select = (tune_param_select > 0) ? tune_param_select - 1 : 2;
                    }
                    else if (key_id == 1) // KEY2:下 
                    {
                        tune_param_select = (tune_param_select < 2) ? tune_param_select + 1 : 0;
                    }
                    else if (key_id == 2) // KEY3: 增加数值
                    {
                        
                        if (tune_param_select == 0) p->Kp += tune_step[0];
                        else if (tune_param_select == 1) p->Ki += tune_step[1];
                        else if (tune_param_select == 2) p->Kd += tune_step[2];
                    }
                    else if (key_id == 3) // KEY4: 减少数值
                    {
                        if (tune_param_select == 0) p->Kp -= tune_step[0];
                        else if (tune_param_select == 1) p->Ki -= tune_step[1];
                        else if (tune_param_select == 2) p->Kd -= tune_step[2];
                    }
                }
                break;
            default:
                break;
            }
        }
         if (key[key_id].Long_Flag)
        {
            key[key_id].Long_Flag = 0; 
            if (key_id == 0)
            {
                if (current_state == MENU_PID_TUNE_MOTOR)
                {
                    // 切换电机
                    tune_motor_select = (tune_motor_select == 0) ? 1 : 0;
                    Menu_resquest = 1; 
                }
            }
            else if(key_id == 3)
            {
                switch(current_state)
                {
                    case MENU_PID_TUNE_MOTOR:
                    case MENU_PID_TUNE_TURN:
                    case MENU_PID_TUNE_ANGLE:
                        current_state = MENU_PID_DEBUG; // 从任何PID调参界面返回到PID调试主菜单
                        Selected_index = 0;
                        Menu_resquest = 1;
                        break;
                    default:
                        break;
                }
            }

            continue; 
        }
    }
}

static void Menu_Update(void) 
{
    char buffer[32];
   if (Get_MaixCAM_Last_Byte() != 0x00)
    {
        current_state = MENU_STATE_START;
    }     
    switch (current_state)
    {
    case MENU_STATE_START:
        {
            uint8_t expected_num = Get_MaixCAM_Expected_Number();
            uint8_t direction = Get_MaixCAM_Direction();
            char dir_str[10];

            if (direction == 0x01) snprintf(dir_str, sizeof(dir_str), "Left ");
            else if (direction == 0x02) snprintf(dir_str, sizeof(dir_str), "Right");
            else snprintf(dir_str, sizeof(dir_str), "---  ");

            snprintf(buffer, sizeof(buffer), "Expect: %d", expected_num);
            OLED_ShowString(0, 16, (uint8_t *)buffer, 16, 1);

            snprintf(buffer, sizeof(buffer), "Direct: %s", dir_str);
            OLED_ShowString(0, 32, (uint8_t *)buffer, 16, 1);

            snprintf(buffer, sizeof(buffer), "F:%-4lu Count:%d", 
                     Get_MaixCAM_Frame_Counter(), 
                     count);
            OLED_ShowString(0, 48, (uint8_t*)buffer, 16, 1);
        }
        break;
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
    case MENU_SHOW_VIEW:
        {
            uint8_t sensor_value = Read_Sensor();
            char view_buffer[17]; // 8个传感器+8个空格+1个结尾符=17
            
            for (int i = 0; i < 8; i++)
            {
                // 从左到右显示(bit 0 -> sensor 1)
                view_buffer[i * 2] = ((sensor_value >> i) & 1) ? '1' : '0';
                view_buffer[i * 2 + 1] = ' ';
            }
            view_buffer[15] = '\0'; 

            OLED_ShowString(4, 24, (uint8_t *)view_buffer, 16, 1);
        }
        break;
    case MENU_PID_TUNE_MOTOR:
    case MENU_PID_TUNE_TURN:
    case MENU_PID_TUNE_ANGLE:
        {
            PID_TypeDef *P = NULL;
            if(current_state == MENU_PID_TUNE_MOTOR) 
            {
                 P = (tune_motor_select == 0) ? &pid_motor1 : ((tune_motor_select == 1) ? &pid_motor2 : &pid_position);
            } 
            else if (current_state == MENU_PID_TUNE_TURN) 
            {
                P = &pid_tuen;
            } 
            else if (current_state == MENU_PID_TUNE_ANGLE) 
            {
                P = &pid_angle;
            }
            
            if(P) {
                snprintf(buffer, sizeof(buffer), "P:%.3f", P->Kp);
                OLED_ShowString(16, 16, (uint8_t *)buffer, 16, 1);
                snprintf(buffer, sizeof(buffer), "I:%.3f", P->Ki);
                OLED_ShowString(16, 32, (uint8_t *)buffer, 16, 1);
                snprintf(buffer, sizeof(buffer), "D:%.3f", P->Kd);
                OLED_ShowString(16, 48, (uint8_t *)buffer, 16, 1);
                OLED_Refresh();
            }
        }      
        break;
    case MENU_SET_SPEED:
        snprintf(buffer, sizeof(buffer), "Target:%.1fcm/s", Target_Speed);
        OLED_ShowString(10, 24, (uint8_t *)buffer, 16, 1);
        break;
    case MENU_SET_ANGLE:
        snprintf(buffer, sizeof(buffer), "Angle:%.1f deg", g_target_angle_setting);
        OLED_ShowString(10, 24, (uint8_t *)buffer, 16, 1);
        break;
    default:
        break;
    }
}

void Menu_Init(void) 
{
    OLED_Init();
}

void Menu_loop(void) 
{
    Key_input();
    if(Menu_resquest)
    {
        display_menu();
        Menu_resquest = 0;
    }
    Menu_Update();
    OLED_Refresh(); // 统一在最后刷新屏幕
}
