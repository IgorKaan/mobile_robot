#include "state_estimation/imu_odom_ekf.h"

imu_odom_ekf::imu_odom_ekf()
{
    m_state = state_vector::Zero();
}

imu_odom_ekf::state_vector imu_odom_ekf::get_state_vector() const
{
    return m_state;
}

void imu_odom_ekf::set_state_vector(const imu_odom_ekf::state_vector& state)
{
    m_state = state;
}

void imu_odom_ekf::predict(const Eigen::Vector3f& velocity_command)
{

}

void imu_odom_ekf::correct(const Eigen::Vector3f& measurement)
{

}

