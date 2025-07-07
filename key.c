#include "key.h"

KEY key[KEY_Number];

void Key_Read(void)
{
    key[0].Down_State = KEY1_STATE;
    key[1].Down_State = KEY2_STATE;
    key[2].Down_State = KEY3_STATE;
    key[3].Down_State = KEY4_STATE;

    
    for(int i=0; i<KEY_Number; i++)
    {
        // 按键状态机 
        switch (key[i].Judge_State)
        {
        case 0: // 状态0: 等待按键按下
            if(key[i].Down_State == 0) //按下key
            {
                key[i].Judge_State = 1; // 进入消抖
                key[i].Down_Time = 0;   // 开始计时
            }
            break;
        case 1: // 状态1: 消抖状态
            if(key[i].Down_State == 0) // 按键仍然按下
            {
                key[i].Judge_State = 2; // 进入按键按下状态
            }
            else // 按键误触
            {
                key[i].Judge_State = 0; // 返回等待状态
            }
            break;
        case 2: // 状态2: 按键按下状态
            if(key[i].Down_State == 1) // 按键松开
            {
                if(key[i].Down_Time < KEY_LONG_PRESS_TIME)//按下时间小于长按阈值，说明是短按或双击中的第一次点击
                {
                    if(key[i].Double_Time_EN==0)
                    {
                        key[i].Double_Time_EN=1; // 启用双击计时
                        key[i].Double_Time = 0; // 重置双击计时
                    }
                }
                else
                {
                    key[i].Double_Flag=1;
                    key[i].Double_Time_EN=0; // 关闭双击计时
                }
                key[i].Judge_State = 0; // 返回等待状态
            }
            else
            {
                key[i].Down_Time++; // 计时
                if(key[i].Down_Time >= KEY_LONG_PRESS_TIME) // 按下时间超过长按阈值
                {
                    key[i].Long_Flag = 1; // 设置长按标志
                }    
            }
            break;
        }

        if(key[i].Double_Time_EN==1)
        {
            key[i].Down_Time++; // 双击计时
            if(key[i].Down_Time>=KEY_DOUBLE_CLICK_INTERVAL)//超时就是短按
            {
                key[i].Short_Flag=1; // 设置短按标志
                key[i].Double_Time_EN=0; // 关闭双击计时
            }
        }
    }
}