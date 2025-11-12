#include "can.h"
#include "main.h"
#include "tim.h"
//
// Created by Sesar on 2025/11/12.
//
extern CAN_RxHeaderTypeDef rx_header;
extern CAN_TxHeaderTypeDef tx_header;
extern uint8_t tx_data[8];
extern uint8_t rx_data[8];
extern uint32_t can_tx_mail_box_;
uint32_t count;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == htim6.Instance)
    {


            HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &can_tx_mail_box_);
            //count++;
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan -> Instance == CAN1)
    {
        //if (rx_header.StdId == 0x205)
        //{
            HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);
        //}
					if(rx_header.StdId == 0x207){
						count++;
					}
    }
}
