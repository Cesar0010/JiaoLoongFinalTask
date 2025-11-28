#include "cmsis_os2.h"
#include "can.h"
#include "iwdg.h"
#include "RCC.h"
#include "tim.h"
#include "usart.h"
extern Rcc rcc;

extern CAN_RxHeaderTypeDef rx_header;
extern uint8_t rx_data[8];
extern osSemaphoreId_t CAN_semaphore_handle;
extern osSemaphoreId_t Rcc_semaphore_handle;
extern uint8_t rx_buffer[18];

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan -> Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);
        osSemaphoreRelease(CAN_semaphore_handle);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart == &huart3)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3,rx_buffer, 18);
        rcc.rx_buffer_ = rx_buffer;
        rcc.last_tick = rcc.tick;
        rcc.tick = HAL_GetTick();
        if (rcc.tick - rcc.last_tick > 1000)
        {
            rcc.connected = false;
        }
        else
        {
            rcc.connected = true;
        }
        osSemaphoreRelease(Rcc_semaphore_handle);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == htim6.Instance)
    {
        HAL_IWDG_Refresh(&hiwdg);
    }
}