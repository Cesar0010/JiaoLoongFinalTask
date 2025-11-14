
#include "imu.h"
#include <cmath>
#include "bmi088.h"


void IMU::acc_calculate()
{
    bmi088_accel_read_reg(0x41, &acc_range, 1);
    bmi088_accel_read_reg(0x12, accel, 6);
    acc_x = accel[0] + accel[1] *256;
    acc_y = accel[2] + accel[3] *256;
    acc_z = accel[4] + accel[5] *256;
    acc_x_ = acc_x/32768.0*10*pow(2,(acc_range+1))*1.5;
    acc_y_ = acc_y/32768.0*10*pow(2,(acc_range+1))*1.5;
    acc_z_ = acc_z/32768.0*10*pow(2,(acc_range+1))*1.5;

    acc_x_= alpha*acc_x_last+acc_x_*(1-alpha);
    acc_y_= alpha*acc_y_last+acc_y_*(1-alpha);
    acc_z_= alpha*acc_z_last+acc_z_*(1-alpha);
    acc_x_last = acc_x_;
    acc_y_last = acc_y_;
    acc_z_last = acc_z_;

}

void IMU::gyro_calculate()
{
    bmi088_gyro_read_reg(0x02, gyro, 6);
    gyro_x = gyro[0] + gyro[1] *256;
    gyro_y = gyro[2] + gyro[3] *256;
    gyro_z = gyro[4] + gyro[5] *256;
    gyro_x_ = gyro_x/32768.0*2000;
    gyro_y_ = gyro_y/32768.0*2000;
    gyro_z_ = gyro_z/32768.0*2000;

    gyro_x_= beta*gyro_x_last+gyro_x_*(1-beta);
    gyro_y_= beta*gyro_y_last+gyro_y_*(1-beta);
    gyro_z_= beta*gyro_z_last+gyro_z_*(1-beta);
    gyro_x_last = gyro_x_;
    gyro_y_last = gyro_y_;
    gyro_z_last = gyro_z_;

}
