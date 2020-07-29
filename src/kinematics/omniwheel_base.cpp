#include "kinematics/omniwheel_base.h"

omniwheel_base::pose_with_twist omniwheel_base::forward_kinematics(const pose2d& pose, const parameters& params, const wheel_velocities& velocities)
{

}

Eigen::MatrixXf omniwheel_base::get_local_jacobian(const parameters& parameters)
{
    float R = parameters.axis_length;
    float theta = parameters.wheel_angle;
    float alpha1 = parameters.top_left_wheel_offset;
    float alpha2 = parameters.bottom_left_wheel_offset;
    float alpha3 = parameters.bottom_right_wheel_offset;
    float alpha4 = parameters.top_right_wheel_offset;
    Eigen::MatrixXf jacobian = Eigen::MatrixXf(4, 3);
    jacobian << std::cos(theta + alpha1), std::sin(theta + alpha1), R,
                std::cos(theta + alpha2), std::sin(theta + alpha2), R,
                std::cos(theta + alpha3), std::sin(theta + alpha3), R,
                std::cos(theta + alpha4), std::sin(theta + alpha4), R;

    return jacobian;
}
