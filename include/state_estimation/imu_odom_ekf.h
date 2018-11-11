#ifndef MOBILE_ROBOT_COMMS_IMU_EKF_H
#define MOBILE_ROBOT_COMMS_IMU_EKF_H

#include <Eigen/Dense>

class imu_odom_ekf {
public:
    using state_vector = Eigen::Matrix<float, 6, 1>;
public:
    imu_odom_ekf();

    state_vector get_state_vector() const;
    void set_state_vector(const state_vector& state);

    void predict(const Eigen::Vector3f& velocity_command);
    void correct(const Eigen::Vector3f& measurement);
private:
    state_vector m_state;
};

#endif //MOBILE_ROBOT_COMMS_IMU_EKF_H
