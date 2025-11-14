//
// Created by Sesar on 2025/11/14.
//
#include "user_tasks.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "motor.h"
#include "can.h"

extern uint8_t rx_data[8];
extern uint8_t tx_data[8];
extern float target_angle_pitch;
extern float target_angle_yaw;
extern CAN_RxHeaderTypeDef rx_header;
extern CAN_TxHeaderTypeDef tx_header;
extern uint32_t can_tx_mail_box_;

Motor motor_pitch(1,0x208,0,0,0,0,0,0,4000,4000,16384,16384,0.1,0.1);
Motor motor_yaw(1,0x205,0,0,0,0,0,0,4000,4000,16384,16384,0.1,0.1);

constexpr auto flag_1 = 1u << 0;
constexpr auto flag_2 = 1u << 1;
constexpr auto flag_3 = 1u << 2;
constexpr auto flag_4 = 1u << 3;
constexpr auto flag_5 = 1u << 4;
constexpr auto flag_6 = 1u << 5;

osSemaphoreAttr_t CAN_semaphore_attributes = {.name = "CAN_semaphore"};
osSemaphoreId_t CAN_semaphore_handle;
osEventFlagsAttr_t event_attributes = {.name = "event"};
osEventFlagsId_t event_handle;

osThreadId_t motor_feedback_decoding_task_handle;
constexpr osThreadAttr_t motor_feedback_decoding_task_attributes = {
    .name = "motor_feedback_decoding_task",
    .stack_size = 128 * 4,
    .priority = (osPriorityNormal),
};

osThreadId_t PID_control_task_handle;
constexpr osThreadAttr_t PID_control_task_attributes = {
    .name = "PID_control_task",
    .stack_size = 128 * 4,
    .priority = (osPriorityLow),
};

osThreadId_t CAN_emmit_task_handle;
constexpr osThreadAttr_t CAN_emmit_task_attributes = {
    .name = "CAN_emmit_task",
    .stack_size = 128 * 4,
    .priority = (osPriorityLow),
};


[[noreturn]] void motor_feedback_decoding_task(void*)
{
    while (1)
    {
        osSemaphoreAcquire(CAN_semaphore_handle, osWaitForever);
        if (rx_header.StdId == 0x208)
        {
            motor_pitch.canRxMsgCallback(rx_data);
        }
        else if (rx_header.StdId == 0x205)
        {
            motor_yaw.canRxMsgCallback(rx_data);
        }
        osEventFlagsSet(event_handle, flag_4);
    }
}

[[noreturn]] void PID_control_task(void*)
{
    while (1)
    {
        osEventFlagsWait(event_handle, flag_4, osFlagsWaitAll, osWaitForever);
        float feedforward_intensity =motor_pitch.FeedforwardIntensityCalc();
        motor_pitch.SetPosition(target_angle_pitch,0.0f,feedforward_intensity);
        motor_pitch.handle();
        motor_pitch.output();
        //motor_yaw.SetPosition(target_angle_yaw,0.0f,0.0f);
        motor_yaw.SetSpeed(0.0f,0.0f);
        motor_yaw.handle();
        motor_yaw.output();
        osEventFlagsSet(event_handle, flag_5);
        osEventFlagsClear(event_handle, flag_4);
    }
}

[[noreturn]] void CAN_emmit_task(void*)
{
    while (1)
    {
        osEventFlagsWait(event_handle, flag_5, osFlagsWaitAll, osWaitForever);
        HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &can_tx_mail_box_);
        osEventFlagsClear(event_handle, flag_5);
    }
}

void user_tasks_init()
{
    CAN_semaphore_handle = osSemaphoreNew(1, 1, &CAN_semaphore_attributes);
    event_handle = osEventFlagsNew(&event_attributes);
    motor_feedback_decoding_task_handle = osThreadNew(motor_feedback_decoding_task, NULL, &motor_feedback_decoding_task_attributes);
    PID_control_task_handle = osThreadNew(PID_control_task, NULL, &PID_control_task_attributes);
    CAN_emmit_task_handle = osThreadNew(CAN_emmit_task, NULL, &CAN_emmit_task_attributes);
}

