//
// Created by Sesar on 2025/11/14.
//
#include "user_tasks.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "motor.h"
#include "can.h"
#include "imu.h"
#include "RCC.h"

extern uint8_t rx_data[8];
extern uint8_t tx_data[8];
extern float target_angle_pitch;
extern float target_angle_yaw;
extern CAN_RxHeaderTypeDef rx_header;
extern CAN_TxHeaderTypeDef tx_header;
extern uint32_t can_tx_mail_box_;

Motor motor_pitch(1,0x208,8,0.01,600,49,0,0,4000,4000,16384,16384,0.06,0.0);
Motor motor_yaw(1,0x205,20,0.001,600,210,0,0,4000,4000,16384,16384,0.07,0.0);
IMU imu;
Rcc rcc;

constexpr auto flag_1 = 1u << 0;
constexpr auto flag_2 = 1u << 1;
constexpr auto flag_3 = 1u << 2;
constexpr auto flag_4 = 1u << 3;
constexpr auto flag_5 = 1u << 4;

osSemaphoreAttr_t CAN_semaphore_attributes = {.name = "CAN_semaphore"};
osSemaphoreId_t CAN_semaphore_handle;
osSemaphoreAttr_t Rcc_semaphore_attributes = {.name = "Rcc_semaphore"};
osSemaphoreId_t Rcc_semaphore_handle;
osEventFlagsAttr_t event_attributes = {.name = "event"};
osEventFlagsId_t event_handle;

osThreadId_t motor_feedback_decoding_task_handle;
constexpr osThreadAttr_t motor_feedback_decoding_task_attributes = {
    .name = "motor_feedback_decoding_task",
    .stack_size = 128 * 8,
    .priority = (osPriorityHigh1),
};

osThreadId_t PID_control_task_handle;
constexpr osThreadAttr_t PID_control_task_attributes = {
    .name = "PID_control_task",
    .stack_size = 128 * 8,
    .priority = (osPriorityLow),
};

osThreadId_t CAN_emmit_task_handle;
constexpr osThreadAttr_t CAN_emmit_task_attributes = {
    .name = "CAN_emmit_task",
    .stack_size = 128 * 4,
    .priority = (osPriorityLow),
};

osThreadId_t IMU_decoding_task_handle;
constexpr osThreadAttr_t IMU_decoding_task_attributes = {
    .name = "IMU_decoding_task",
    .stack_size = 128 * 8,
    .priority = (osPriorityNormal),
};

osThreadId_t Rcc_decoding_task_handle;
constexpr osThreadAttr_t Rcc_decoding_task_attributes = {
    .name = "Rcc_decoding_task",
    .stack_size = 128 * 8,
    .priority = (osPriorityHigh),
};

osThreadId_t Stop_motor_task_handle;
constexpr osThreadAttr_t Stop_motor_task_attributes = {
    .name = "Stop_motor_task",
    .stack_size = 128 * 4,
    .priority = (osPriorityHigh),
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
        osEventFlagsWait(event_handle, flag_4, osFlagsWaitAny, osWaitForever);
        float feedforward_intensity =motor_pitch.FeedforwardIntensityCalc();
        motor_pitch.SetPosition(target_angle_pitch,0.0f,feedforward_intensity);
        //motor_pitch.SetSpeed(0,feedforward_intensity);
        motor_pitch.handle();
        motor_pitch.output();

        //motor_yaw.SetPosition(target_angle_yaw,0.0f,0.0f);
        motor_yaw.SetSpeed(0,0);
        motor_yaw.handle();
        //motor_yaw.output();
        osEventFlagsClear(event_handle, flag_3);
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

[[noreturn]] void IMU_decoding_task(void*)
{
    while (1)
    {
        imu.acc_calculate();
        imu.gyro_calculate();
        osDelay(1);
    }
}

[[noreturn]] void Rcc_decoding_task(void*)
{
    while (1)
    {
        osSemaphoreAcquire(Rcc_semaphore_handle, osWaitForever);
        rcc.handle();
        if (rcc.read_remote.S1_toggle == Rcc::read_remote_::S1_UP)
        {
            osEventFlagsSet(event_handle, flag_2);
        }
        else if (rcc.read_remote.S1_toggle == Rcc::read_remote_::S1_DOWN)
        {
            osEventFlagsSet(event_handle, flag_1);
            osEventFlagsClear(event_handle, flag_2);
        }
        //target_angle_pitch = rcc.read_remote.channel_1 * 180;
        //target_angle_yaw = rcc.read_remote.channel_0 * 180;
        osEventFlagsSet(event_handle, flag_3);
    }
}

[[noreturn]] void Stop_motor_task(void*)
{
    while (1)
    {
        osEventFlagsWait(event_handle, flag_1, osFlagsWaitAll, osWaitForever);
        uint8_t stop_data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        HAL_CAN_AddTxMessage(&hcan1, &tx_header, stop_data, &can_tx_mail_box_);
        osEventFlagsClear(event_handle, flag_1);
    }
}

void user_tasks_init()
{
    CAN_semaphore_handle = osSemaphoreNew(1, 1, &CAN_semaphore_attributes);
    Rcc_semaphore_handle = osSemaphoreNew(1, 1, &Rcc_semaphore_attributes);
    event_handle = osEventFlagsNew(&event_attributes);
    motor_feedback_decoding_task_handle = osThreadNew(motor_feedback_decoding_task, NULL, &motor_feedback_decoding_task_attributes);
    PID_control_task_handle = osThreadNew(PID_control_task, NULL, &PID_control_task_attributes);
    CAN_emmit_task_handle = osThreadNew(CAN_emmit_task, NULL, &CAN_emmit_task_attributes);
    IMU_decoding_task_handle = osThreadNew(IMU_decoding_task, NULL, &IMU_decoding_task_attributes);
    Rcc_decoding_task_handle = osThreadNew(Rcc_decoding_task, NULL, &Rcc_decoding_task_attributes);
    Stop_motor_task_handle = osThreadNew(Stop_motor_task, NULL, &Stop_motor_task_attributes);
}


