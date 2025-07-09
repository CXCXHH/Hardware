#include "MaixCAM.h"
#include "ti_msp_dl_config.h" 


/*================================================================================*/
// 模块内部静态变量
/*================================================================================*/
// 用于存储MaixCAM接收数据的缓冲区
static uint8_t g_ucMaixCAMRxBuff[MAIXCAM_RX_BUFF_SIZE] = {0}; 
// 数据接收标志位
static uint8_t g_ucDataFlag = 0; 
// 接收数据计数器
static uint8_t g_ucRxCount = 0;  

// 存储坐标信息
static uint8_t g_ucCoordinate = 0;
// 存储识别的数字信息
static uint8_t g_ucNumber = 0;

/*================================================================================*/
// 内部函数声明
/*================================================================================*/
static uint8_t MaixCAM_Data_Parse(uint8_t *pack);

/*================================================================================*/
// 中断服务函数
/*================================================================================*/
// MaixCAM串口中断服务函数 (对应UART1)
void MaixCAM_INST_IRQHandler(void)
{
    uint8_t received_data;

    switch (DL_UART_Main_getPendingInterrupt(MaixCAM_INST)) 
    {
        case DL_UART_MAIN_IIDX_RX:
            received_data = DL_UART_Main_receiveData(MaixCAM_INST);

            // 检查帧头
            if (g_ucDataFlag == 0 && received_data == 0x6b) 
            {
                g_ucDataFlag = 1;
                g_ucRxCount = 0;
            }
            // 接收数据
            else if (g_ucDataFlag == 1) 
            {
                if (g_ucRxCount < MAIXCAM_RX_BUFF_SIZE) 
                {
                    g_ucMaixCAMRxBuff[g_ucRxCount++] = received_data;
                }
                
                // 帧接收完成
                if (g_ucRxCount >= MAIXCAM_RX_BUFF_SIZE)
                {
                    g_ucDataFlag = 0;
                    // 解析数据
                    if (MaixCAM_Data_Parse(g_ucMaixCAMRxBuff))
                    {
                        g_ucCoordinate = 0; 
                        g_ucNumber = g_ucMaixCAMRxBuff[2];
                    }
                }
            }
            break;
        default:
            break;
    }
}

/*================================================================================*/
// 模块功能函数
/*================================================================================*/
// 解析数据包
static uint8_t MaixCAM_Data_Parse(uint8_t *pack)
{
    // 检查协议帧头和帧尾
    if (pack[0] != 0x5b) return 0;
    if (pack[1] != 0x5b) return 0;
    // [修复] 缓冲区大小为4, 最后一个字节的索引是3
    if (pack[MAIXCAM_RX_BUFF_SIZE - 1] != 0xb3) return 0;
    return 1;
}

// MaixCAM模块初始化
void MaixCAM_Init(void)
{
    // 清除并使能MaixCAM对应的串口中断 (UART1)
    NVIC_ClearPendingIRQ(MaixCAM_INST_INT_IRQN);
    NVIC_EnableIRQ(MaixCAM_INST_INT_IRQN);
}

// 获取坐标信息
uint8_t Get_MaixCAM_Coordinate(void)
{
    return g_ucCoordinate;
}

// 获取识别的数字
uint8_t Get_MaixCAM_Number(void)
{
    return g_ucNumber;
}

/**
 * @brief  手动清除内部存储的识别数字
 * @note   用于在任务开始时清除旧的、可能残留的数字，避免误判
 * @param  None
 * @retval None
 */
void MaixCAM_Clear_Number(void)
{
    g_ucNumber = 0;
}
