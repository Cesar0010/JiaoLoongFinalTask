#include <stdint.h>
#include "pid.h"
class Motor
{
    private:
    const float ratio_;
    float angle_;
    float delta_angle_;
    float ecd_angle_;
    float last_ecd_angle_;
    float delta_ecd_angle_;
    float rotate_speed_;
    float current_;
    float temp_;
    bool init_flag_;
    PID spid_, ppid_;
    float target_angle_, fdb_angle_;
    float target_speed_, fdb_speed_, feedforward_speed_;
    float feedforward_intensity_, output_intensity_;
    enum {
        TORQUE,
        SPEED,
        POSITION_SPEED,
      } control_method_;
public:
    void SetPosition(float target_position, float feedforward_speed, float feedforward_intensity);
    void SetSpeed(float target_speed, float feedforward_intensity);
    void SetIntensity(float intensity);
    float FeedforwardIntensityCalc();
    Motor(const float ratio, float p_kp, float p_ki, float p_kd, float s_kp, float s_ki, float s_kd,
    float p_imax, float s_imax, float p_out_max, float s_out_max, float p_d_filter_k, float s_d_filter_k)
    : ratio_(ratio),
      spid_(s_kp, s_ki, s_kd, s_imax, s_out_max, s_d_filter_k),
      ppid_(p_kp, p_ki, p_kd, p_imax, p_out_max, p_d_filter_k)
    {
        angle_ = 0.0f;
        ecd_angle_ = 0.0f;
        last_ecd_angle_ = 0.0f;
        delta_ecd_angle_ = 0.0f;
        delta_angle_ = 0.0f;
        rotate_speed_ = 0.0f;
        current_ = 0.0f;
        temp_ = 0.0f;
        target_angle_ = 0.0f;
        target_speed_ = 0.0f;
        fdb_angle_ = 0.0f;
        fdb_speed_ = 0.0f;
        feedforward_speed_ = 0.0f;
        feedforward_intensity_ = 0.0f;
        output_intensity_ = 0.0f;
        init_flag_ = true;
        control_method_ = TORQUE;
    }
    void canRxMsgCallback(const uint8_t rx_data[8]);
    void handle();
    void output();
};

