#include <utility>

#include "imu.h"

using namespace platform_imu;

imu::imu(const std::string bus) : bus_(std::move(bus)){
    fd = 0;
    for (size_t i = 0 ; i < num_op_reg; ++i){
        gyro_data[i] = 0;
        accel_data[i] = 0;
        magn_data[i] = 0;
    }
}

void imu::init() {
    if((fd = open(bus_.c_str(), O_RDWR)) < 0) {
        std::cerr << "Failed to open the bus.\n";
        exit(1);
    }
    init_gyro();
    init_accel();
    init_magn();
}

void imu::init_gyro() {
    ioctl(fd, I2C_SLAVE, L3G4200D_ADRESS);

    uint8_t config[2] = {0};
    config[0] = CTRL_REG1;
    config[1] = 0x0F;
    write(fd, config, sizeof(config));

    config[0] = CTRL_REG4;
    config[1] = 0x30;
    write(fd, config, sizeof(config));
}

void imu::init_accel() {
    ioctl(fd, I2C_SLAVE, LIS331DLH_ADRESS);

    uint8_t config[2] = {0};
    config[0] = CTRL_REG1;
    config[1] = 0x2F;
    write(fd, config, sizeof(config));

    config[0] = CTRL_REG4;
    config[1] = 0x80;
    write(fd, config, sizeof(config));
}

void imu::init_magn() {
    ioctl(fd, I2C_SLAVE, LIS3MDL_ADRESS);
    uint8_t config[num_op_reg] = {CTRL_REG1, 0x5C, 0x00, 0x00, 0x08, 0x40};
    write(fd, config, sizeof(config));
}

void imu::read_gyro_data() {
    ioctl(fd, I2C_SLAVE, L3G4200D_ADRESS);
    read_reg(gyro_data);
}

void imu::read_accel_data() {
    ioctl(fd, I2C_SLAVE, LIS331DLH_ADRESS);
    read_reg(accel_data);
}

void imu::read_magn_data() {
    ioctl(fd, I2C_SLAVE, LIS3MDL_ADRESS);
    read_reg(magn_data);
}

void imu::read_reg(float *data) {
    char reg[num_op_reg] = { REG_OUT_X_L, REG_OUT_X_H,
                             REG_OUT_Y_L, REG_OUT_Y_H,
                             REG_OUT_Z_L, REG_OUT_Z_H };

    for (size_t i = 0; i < num_op_reg; ++i){
        write(fd, &reg[i], sizeof(reg[i]));
        read(fd, &reg[i], sizeof(reg[i]));
        data[i] = reg[i];
    }
}

void imu::read_data() {
    read_gyro_data();
    read_accel_data();
    std::cout << pos.angular_velocity.x << " ";
    std::cout << pos.angular_velocity.y << " ";
    std::cout << pos.angular_velocity.z << "\n";
    //read_magn_data();
}

void imu::convert_gyro_data_deg_per_sec() {
    pos.angular_velocity.x = (gyro_data[1] * 256 + gyro_data[0]) * SENS_FS_250;
    pos.angular_velocity.y = (gyro_data[3] * 256 + gyro_data[2]) * SENS_FS_250;
    pos.angular_velocity.z = (gyro_data[5] * 256 + gyro_data[4]) * SENS_FS_250;
}

void imu::convert_gyro_data_rad_per_sec() {
    pos.angular_velocity.x = ((gyro_data[1] * 256 + gyro_data[0]) * SENS_FS_250)*(M_PI/180.0);
    pos.angular_velocity.y = ((gyro_data[3] * 256 + gyro_data[2]) * SENS_FS_250)*(M_PI/180.0);
    pos.angular_velocity.z = ((gyro_data[5] * 256 + gyro_data[4]) * SENS_FS_250)*(M_PI/180.0);
}

void imu::convert_accel_data_g() {
    pos.linear_acceleration.x = ((accel_data[1] * 256 + accel_data[0]) * RANGE_2G / 32767.0);
    pos.linear_acceleration.y = ((accel_data[3] * 256 + accel_data[2]) * RANGE_2G / 32767.0);
    pos.linear_acceleration.z = ((accel_data[5] * 256 + accel_data[4]) * RANGE_2G / 32767.0);
}

// Accelerometr data in m/s^2
void imu::convert_accel_data() {
    pos.linear_acceleration.x = (((accel_data[1] * 256 + accel_data[0]) * RANGE_2G / 32767.0) * G);
    pos.linear_acceleration.y = (((accel_data[3] * 256 + accel_data[2]) * RANGE_2G / 32767.0) * G);
    pos.linear_acceleration.z = (((accel_data[5] * 256 + accel_data[4]) * RANGE_2G / 32767.0) * G);
}

void imu::convert_magn_data() {
    const float compass_calibrated_bias[3] = {
            524.21f,
            3352.214f,
            -1402.236f
    };

    const float compass_calibrated_matrix[3][3] = {
            {1.757f, 0.04f, -0.028f},
            {0.008f, 1.767f, -0.016f},
            {-0.018f, 0.077f, 1.782f}
    };

    float uncalibrated_data[3];
    uncalibrated_data[0] = ((magn_data[1] * 256 + magn_data[0])/SENS_FS_4);
    uncalibrated_data[1] = ((magn_data[3] * 256 + magn_data[2])/SENS_FS_4);
    uncalibrated_data[2] = ((magn_data[5] * 256 + magn_data[4])/SENS_FS_4);

    float uncalibrated_values[3];
    uncalibrated_values[0] = uncalibrated_data[0] - compass_calibrated_bias[0];
    uncalibrated_values[1] = uncalibrated_data[1] - compass_calibrated_bias[1];
    uncalibrated_values[2] = uncalibrated_data[2] - compass_calibrated_bias[2];

    float calibrated_values[3];
    for (size_t i = 0; i < 3; ++i)
        for (size_t j = 0; j < 3; ++j)
            calibrated_values[i] += compass_calibrated_matrix[i][j] * uncalibrated_values[j];

    //pos.magn.x = (calibrated_values[0]/SENS_FS_4);
    //pos.magn.y = (calibrated_values[1]/SENS_FS_4);
    //pos.magn.z = (calibrated_values[2]/SENS_FS_4);
}


void imu::convert_data() {
    convert_gyro_data_rad_per_sec();
    convert_accel_data();
    //convert_magn_data();
}

const sensor_msgs::Imu &imu::get_pos() const {
    return pos;
}
