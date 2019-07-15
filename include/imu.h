#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>

#include <sstream>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <iostream>

#define L3G4200D_ADRESS   0x68
#define LIS331DLH_ADRESS  0x18
#define LIS3MDL_ADRESS    0x1C

#define CTRL_REG1         0x20
#define CTRL_REG4         0x23

#define REG_OUT_X_L       0x28
#define REG_OUT_X_H       0x29
#define REG_OUT_Y_L       0x2A
#define REG_OUT_Y_H       0x2B
#define REG_OUT_Z_L       0x2C
#define REG_OUT_Z_H       0x2D

namespace platform_imu {
    const float SENS_FS_4 = 6842.0f;
    const float SENS_FS_250 = 0.00875f;
    const float SENS_FS_500 = 0.0175f;
    const float SENS_FS_2000 = 0.07f;
    const float SENS_FS_CURR = SENS_FS_250;
    const float RANGE_2G = 2;
    const float G = 9.81f;
    const int num_op_reg = 6;    // number of output x/y/z registers

    class imu {
    private:
        int fd;
        std::string bus_;
        float gyro_data[num_op_reg];
        float accel_data[num_op_reg];
        float magn_data[num_op_reg];

        sensor_msgs::Imu pos;

        void init_gyro();
        void init_accel();
        void init_magn();
        void read_reg(float *data);
        void read_gyro_data();
        void read_accel_data();
        void read_magn_data();
        void convert_gyro_data_deg_per_sec();
        void convert_gyro_data_rad_per_sec();
        void convert_accel_data_g();
        void convert_accel_data();
        void convert_magn_data();

    public:
        explicit imu(std::string bus);

        void init();
        void read_data();
        void convert_data();

        const sensor_msgs::Imu& get_pos() const;
    };

}
