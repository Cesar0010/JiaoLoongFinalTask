#include "cmsis_os2.h"
#include "can.h"

extern CAN_RxHeaderTypeDef rx_header;
extern uint8_t rx_data[8];
extern osSemaphoreId_t CAN_semaphore_handle;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan -> Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);
        osSemaphoreRelease(CAN_semaphore_handle);
    }
}
