#include "MaixCAM.h"
#include "ti_msp_dl_config.h"

static uint8_t g_ucMaixCAMRxBuff[MAIXCAM_RX_BUFF_SIZE] = {0};
static uint8_t g_ucDataFlag = 0;
static uint8_t g_ucRxCount = 0;
static volatile uint8_t g_ucDirection = 0;
static volatile uint8_t g_ucExpectedNumber = 0;
static volatile uint32_t g_ulValidFrameCounter = 0;
static volatile uint8_t g_ucLastReceivedByte = 0;

static uint8_t MaixCAM_Data_Parse(uint8_t *pack);

void MaixCAM_INST_IRQHandler(void)
{

    
    uint8_t received_data;
    switch (DL_UART_Main_getPendingInterrupt(MaixCAM_INST))
    {
        case DL_UART_MAIN_IIDX_RX:
            received_data = DL_UART_Main_receiveData(MaixCAM_INST);
            g_ucLastReceivedByte = received_data;
            if (g_ucDataFlag == 0 && received_data == 0x6b)
            {
                g_ucDataFlag = 1;
                g_ucRxCount = 0;
            }
            else if (g_ucDataFlag == 1)
            {
                if (g_ucRxCount < MAIXCAM_RX_BUFF_SIZE)
                {
                    g_ucMaixCAMRxBuff[g_ucRxCount++] = received_data;
                }
                if (g_ucRxCount >= MAIXCAM_RX_BUFF_SIZE)
                {
                    g_ucDataFlag = 0;
                    if (MaixCAM_Data_Parse(g_ucMaixCAMRxBuff))
                    {
                        g_ulValidFrameCounter++;
                        uint8_t cmd = g_ucMaixCAMRxBuff[2];
                        if (cmd >= 0x11 && cmd <= 0x18)
                        {
                            g_ucExpectedNumber = cmd - 0x10;
                            g_ucDirection = 0;
                        }
                        else if (cmd == 0x01 || cmd == 0x02)
                        {
                            g_ucDirection = cmd;
                        }
                    }
                }
            }
            break;
        default:
            break;
    }
}

static uint8_t MaixCAM_Data_Parse(uint8_t *pack)
{
    if (pack[0] != 0x5b || pack[1] != 0x5b || pack[3] != 0xb3) return 0;
    return 1;
}

void MaixCAM_Init(void)
{
    NVIC_ClearPendingIRQ(MaixCAM_INST_INT_IRQN);
    NVIC_EnableIRQ(MaixCAM_INST_INT_IRQN);
}

uint8_t Get_MaixCAM_Direction(void) 
{ 
    return g_ucDirection; 
}

uint8_t Get_MaixCAM_Expected_Number(void) 
{ 
    return g_ucExpectedNumber; 
}
void MaixCAM_Send_Command(uint8_t command)
{
    while(DL_UART_isTXFIFOFull(MaixCAM_INST));
    DL_UART_Main_transmitData(MaixCAM_INST, command);
}
uint32_t Get_MaixCAM_Frame_Counter(void) 
{ 
    return g_ulValidFrameCounter; 
}
uint8_t Get_MaixCAM_Last_Byte(void) 
{ 
    return g_ucLastReceivedByte; 
}

uint32_t Get_MaixCAM_Frame_Counter_Reset(void) 
{ 
    return g_ulValidFrameCounter=0; 
}
