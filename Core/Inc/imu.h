
#ifndef BMI088_IMU_H
#define BMI088_IMU_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
class IMU
{
    public:
        void acc_calculate();
        void gyro_calculate();
    private:
        int16_t acc_x, acc_y, acc_z;
        int16_t gyro_x, gyro_y, gyro_z;
        float acc_x_, acc_y_, acc_z_;
        float gyro_x_, gyro_y_, gyro_z_;
        unsigned char acc_range;
        uint8_t accel[6];
        uint8_t gyro[6];
        float alpha = 0.98;
        float beta = 0.98;
        float acc_x_last, acc_y_last, acc_z_last;
        float gyro_x_last, gyro_y_last, gyro_z_last;
};
#ifdef __cplusplus
}
#endif
#endif //BMI088_IMU_H