#ifndef MOBILE_ROBOT_COMMS_IMU_EKF_H
#define MOBILE_ROBOT_COMMS_IMU_EKF_H

#include <Eigen/Dense>

class imu_odom_ekf {
public:
    using vec6f = Eigen::Matrix<float, 6, 1>;
    using mat6f = Eigen::Matrix<float, 6, 6>;
public:
    imu_odom_ekf();

    vec6f get_state_mean() const;
    void set_state_mean(const vec6f& state_mean);

    mat6f get_state_covariance() const;
    void set_state_covariance(const mat6f& state_covariance);

    void predict(const Eigen::Vector3f& velocity_command);
    void correct(const Eigen::Vector3f& measurement);
private:
    vec6f motion_prediction(const vec6f& old_state, const Eigen::Vector3f& velocity_command);
    mat6f motion_jacobian(const vec6f& old_state, const Eigen::Vector3f& velocity_command);
    vec6f imu_measurement(const vec6f& predicted_state, const Eigen::Vector3f& measurement);
    mat6f measurement_jacobian(const vec6f& predicted_state, const Eigen::Vector3f& measurement);

    vec6f m_mean;
    mat6f m_cov;
    mat6f m_prediction_noise;
    mat6f m_measurement_noise;
};

#endif //MOBILE_ROBOT_COMMS_IMU_EKF_H
