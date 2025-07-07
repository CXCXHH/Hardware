#include "Menu.h"
#include "oled.h"
#include "key.h"
#include "Encoder.h"
#include "jy61p.h"

//菜单状态结构体
typedef enum 
{
    MENU_STATE_MAIN,   //主菜单状态
    MENU_DATA_VIEW,   //数据查看状态
    MENU_PARAM_SET,  //参数设置状态
    MENU_SHOW_ENCODER, //显示编码器状态
    MENU_SHOW_GYRO,    //显示陀螺仪状态
    MENU_SHOW_VIEW,    //显示视图状态
} MenuState;



static uint8_t Menu_resquest = 1;// 菜单请求标志
static MenuState current_state = MENU_STATE_MAIN; // 当前菜单状态
static uint8_t Selected_index = 0; // 当前选中菜单项索引

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
    case MENU_SHOW_GYRO:
        OLED_ShowString(0, 0, (uint8_t *)"---Show Gyro---", 16, 1);
        break;
    }
    
}

static void Key_input(void) 
{
    for(int i=0;i<KEY_Number;i++)
    {
        if(key[i].Short_Flag)
        {
            key[i].Short_Flag=0;
            int key_id=i;

            switch (current_state)
            {
                case MENU_STATE_MAIN:
                case MENU_DATA_VIEW:
                case MENU_PARAM_SET:
                    Menu_resquest = 1; // 请求菜单更新
                    if(current_state == MENU_STATE_MAIN)
                    {
                        const int max_index = 2; // 主菜单最大索引
                        if(key_id == 0)//KEY1
                        {
                            if(Selected_index>0)
                            {
                                Selected_index--; // 向下移动
                            }
                            else
                            {
                                Selected_index = max_index; // 回到最后一个选项
                            }
                        } 
                        else if(key_id == 1)//KEY2
                        {
                            if(Selected_index < max_index)
                            {
                                Selected_index++; // 向上移动
                            }
                            else
                            {
                                Selected_index = 0; // 回到第一个选项
                            }
                        } 
                        else if(key_id == 2)//KEY3
                        {
                            switch (Selected_index)
                            {
                                case 0: // Start
                                    current_state = MENU_STATE_MAIN; // 返回主菜单
                                    break;
                                case 1: // Data View
                                    current_state = MENU_DATA_VIEW; // 进入数据查看状态
                                    Selected_index = 0; 
                                    break;
                                case 2: // Param Set
                                    current_state = MENU_PARAM_SET; // 进入参数设置状态
                                    Selected_index = 0;
                                    break;
                            }
                        }
                    }
                    else if(current_state == MENU_DATA_VIEW)
                    {
                        const int max_index = 2; // 数据查看最大索引
                        if(key_id == 0)//KEY1
                        {
                            if(Selected_index>0)
                            {
                                Selected_index--; // 向下移动
                            }
                            else
                            {
                                Selected_index = max_index; // 回到最后一个选项
                            }
                        }
                        else if(key_id == 1)//KEY2
                        {
                            if(Selected_index < max_index)
                            {
                                Selected_index++; // 向上移动
                            }
                            else
                            {
                                Selected_index = 0; // 回到第一个选项
                            }
                        }
                        else if(key_id == 2)//KEY3
                        {
                            switch (Selected_index)
                            {
                                case 0: // Show Encoder
                                    current_state = MENU_SHOW_ENCODER; // 进入显示编码器状态
                                    break;
                                case 1: // Show Gyro
                                    current_state = MENU_SHOW_GYRO; // 进入显示陀螺仪状态
                                    break;
                                case 2: // Show view
                                    break;
                            }
                        }
                        else if (key_id == 3)//KEY4
                        {
                            current_state = MENU_STATE_MAIN; // 返回主菜单
                        }                      
                    }     
                break;
                case MENU_SHOW_ENCODER:
                    if(key_id == 0)//KEY1
                    {
                        Encoder_Reset_Distance(); // 重置编码器距离
                    }
                    else if(key_id == 3)//KEY4
                    {
                        current_state = MENU_DATA_VIEW; // 返回数据查看状态
                        Selected_index = 0; // 重置选中索引
                        Menu_resquest = 1; // 菜单更新
                    }
                break;
                case MENU_SHOW_GYRO:
                    if(key_id == 3)//KEY4 
                    {
                        current_state = MENU_DATA_VIEW; // 返回数据查看状态
                        Selected_index = 0; // 重置选中索引
                        Menu_resquest = 1; // 菜单更新
                    }   
            }
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
            OLED_ShowString(0, 16, (uint8_t *)buffer, 16, 1);
            snprintf(buffer, sizeof(buffer), "Pitch: %-6.2f", gyro_data->y);
            OLED_ShowString(0, 32, (uint8_t *)buffer, 16, 1);
            snprintf(buffer, sizeof(buffer), "Yaw: %-6.2f", gyro_data->z);
            OLED_ShowString(0, 48, (uint8_t *)buffer, 16, 1);
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
