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

    ekf.predict(Eigen::Vector3f::Zero());
    ekf.correct(Eigen::Vector3f::Zero());

    std::cout << ekf.get_state_vector() << '\n';

    return 0;
}