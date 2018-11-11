#include <iostream>
#include <cerrno>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#include "kinematics/differential_drive.h"
#include "state_estimation/imu_odom_ekf.h"

int main()
{
    imu_odom_ekf ekf;


    for (int i = 0; i < 100; i++) {
        ekf.predict(Eigen::Vector3f::Zero());
        ekf.correct(Eigen::Vector3f {0.2f, 0.3f, 0.5f});
    }

    std::cout << ekf.get_state_mean() << '\n';
    std::cout << ekf.get_state_covariance() << '\n';

    return 0;
}