//
// Created by Sesar on 2025/10/20.
//
#include "motor.h"

#include <cmath>
extern uint8_t tx_data[8];
extern int ttt;
float linearMapping(float input, float input_min, float input_max, float output_min, float output_max)
{
    return (input - input_min) * (output_max - output_min) / (input_max - input_min) + output_min;
}

void Motor::canRxMsgCallback(const uint8_t rx_data[8])
{
    last_ecd_angle_ = ecd_angle_;
    ecd_angle_ = (rx_data[0] << 8) | rx_data[1];
    ecd_angle_=linearMapping(ecd_angle_, 0.0, 8191.0, 0.0, 360.0);
    delta_ecd_angle_ = ecd_angle_ - last_ecd_angle_;
    if (delta_ecd_angle_ > 180.0f) {
        delta_ecd_angle_ -= 360.0f;
    } else if (delta_ecd_angle_ < -180.0f) {
        delta_ecd_angle_ += 360.0f;
    }
    delta_angle_ = delta_ecd_angle_ /ratio_;
    angle_ += delta_angle_;
    int16_t speed_raw = (rx_data[2] << 8) | rx_data[3];
    rotate_speed_ = static_cast<float>(speed_raw);
    int16_t current_raw = (rx_data[4] << 8) | rx_data[5];
    current_raw = static_cast<float>(current_raw);
    ttt = current_raw;
    current_ =linearMapping(current_raw, -16384.0, 16384.0, -3, 3);
    temp_ = rx_data[6];
    if (init_flag_ == true)
    {
        init_flag_ = false;
        angle_ = 0.0;
    }
    fdb_angle_ = angle_;
    fdb_speed_ = rotate_speed_;
}

void Motor::SetPosition(float target_position, float feedforward_speed, float feedforward_intensity)
{
    target_angle_ = target_position;
    feedforward_speed_ = feedforward_speed;
    feedforward_intensity_ = feedforward_intensity;
    control_method_ = POSITION_SPEED;
}

void Motor::SetSpeed(float target_speed, float feedforward_intensity)
{
    target_speed_ = target_speed;
    feedforward_intensity_ = feedforward_intensity;
    control_method_ = SPEED;
}

void Motor::SetIntensity(float intensity)
{
    feedforward_intensity_ = intensity;
    control_method_ = TORQUE;
}

void Motor::handle()
{
    switch (control_method_)
    {
        case TORQUE:
            {
                output_intensity_ = feedforward_intensity_;
                break;
            }
        case SPEED:
            {
                float intensity_from_speed = spid_.calc(target_speed_,fdb_speed_);
                output_intensity_ = intensity_from_speed + feedforward_intensity_;
                break;
            }
        case POSITION_SPEED:
            {
                float speed_from_position = ppid_.calc(target_angle_,fdb_angle_);
                target_speed_ = speed_from_position + feedforward_speed_;
                float intensity_from_speed = spid_.calc(target_speed_,fdb_speed_);
                output_intensity_ = intensity_from_speed + feedforward_intensity_;
            }
    }
}

float Motor::FeedforwardIntensityCalc()
{
    feedforward_intensity_ = 0.5*9.8*sin(angle_/180*3.14159265358979323846)*0.05524/0.3*16384/20;
    float feedforward_intensity = feedforward_intensity_;
    return feedforward_intensity;
}

void Motor::output()
{
    int intensity = static_cast<int>(output_intensity_);
    tx_data[6] = (intensity >> 8) & 0xFF;
    tx_data[7] = intensity & 0xFF;
}


