#include <iostream>

#include "state_estimation/imu_odom_ekf.h"

imu_odom_ekf::imu_odom_ekf()
{
    m_mean = vec6f::Zero();
    m_cov = mat6f::Zero();

    m_prediction_noise = mat6f::Identity() * 0.1f;
    m_measurement_noise = mat6f::Identity() * 0.01f;
}

imu_odom_ekf::vec6f imu_odom_ekf::get_state_mean() const
{
    return m_mean;
}

void imu_odom_ekf::set_state_mean(const imu_odom_ekf::vec6f& state_mean)
{
    m_mean = state_mean;
}

imu_odom_ekf::mat6f imu_odom_ekf::get_state_covariance() const
{
    return m_cov;
}

void imu_odom_ekf::set_state_covariance(const imu_odom_ekf::mat6f& state_covariance)
{
    m_cov = state_covariance;
}

void imu_odom_ekf::predict(const Eigen::Vector3f& velocity_command)
{
    mat6f G = motion_jacobian(m_mean, velocity_command);
    m_mean = motion_prediction(m_mean, velocity_command);
    m_cov = G * m_cov * G.transpose() + m_prediction_noise;
}

void imu_odom_ekf::correct(const Eigen::Vector3f& measurement)
{
    mat6f H = measurement_jacobian(m_mean, measurement);
    mat6f invert_me = H*m_cov*H.transpose() + m_measurement_noise;
    mat6f kalman_gain = m_cov * H.transpose() * invert_me.inverse();

    vec6f meas = vec6f::Zero();
    meas(3) = measurement(0);
    meas(4) = measurement(1);
    meas(5) = measurement(2);

    m_mean = m_mean + kalman_gain * (meas - imu_measurement(m_mean, measurement));
    m_cov = (mat6f::Identity() - kalman_gain * H) * m_cov;
}

imu_odom_ekf::vec6f imu_odom_ekf::motion_prediction(const imu_odom_ekf::vec6f& old_state, const Eigen::Vector3f& velocity_command)
{
    return old_state;
}

imu_odom_ekf::mat6f imu_odom_ekf::motion_jacobian(const imu_odom_ekf::vec6f& old_state, const Eigen::Vector3f& velocity_command)
{
    return imu_odom_ekf::mat6f::Identity();
}

imu_odom_ekf::vec6f imu_odom_ekf::imu_measurement(const imu_odom_ekf::vec6f& predicted_state, const Eigen::Vector3f& measurement)
{
    vec6f meas = vec6f::Zero();
    meas(3) = measurement(0);
    meas(4) = measurement(1);
    meas(5) = measurement(2);
    return meas;
}

imu_odom_ekf::mat6f imu_odom_ekf::measurement_jacobian(const imu_odom_ekf::vec6f& predicted_state, const Eigen::Vector3f& measurement)
{
    return imu_odom_ekf::mat6f::Identity();
}

