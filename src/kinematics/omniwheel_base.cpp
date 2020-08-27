#include "kinematics/omniwheel_base.h"

#include <Eigen/Dense>


// Definition is required outside the class to avoid undefined references;
constexpr float omniwheel_base::DEFAULT_WHEEL_RADIUS;
constexpr float omniwheel_base::DEFAULT_AXIS_LENGTH;
constexpr float omniwheel_base::DEFAULT_WHEEL_ANGLE;
constexpr float omniwheel_base::DEFAULT_TOP_LEFT_WHEEL_OFFSET;
constexpr float omniwheel_base::DEFAULT_BOTTOM_LEFT_WHEEL_OFFSET;
constexpr float omniwheel_base::DEFAULT_BOTTOM_RIGHT_WHEEL_OFFSET;
constexpr float omniwheel_base::DEFAULT_TOP_RIGHT_WHEEL_OFFSET;

omniwheel_base::pose_with_twist omniwheel_base::forward_kinematics(const pose2d& pose, const parameters& params, const wheel_velocities& velocities)
{

}

Eigen::MatrixXf omniwheel_base::get_local_jacobian(const parameters& parameters)
{
    float R = parameters.axis_length;
    float theta = parameters.wheel_angle;
    float wheel_radius = parameters.wheel_radius;
    float alpha1 = parameters.top_left_wheel_offset;
    float alpha2 = parameters.bottom_left_wheel_offset;
    float alpha3 = parameters.bottom_right_wheel_offset;
    float alpha4 = parameters.top_right_wheel_offset;
    Eigen::MatrixXf jacobian = Eigen::MatrixXf(4, 3);
    jacobian << 1, 0, -R,
                0, 1, -R,
                1, 0, R,
                0, 1, R;

    Eigen::MatrixXf rot_mat = Eigen::MatrixXf(3, 3);
    float rot_angle = -omniwheel_base::DEFAULT_WHEEL_ANGLE;
    rot_mat << cos(rot_angle), sin(rot_angle), 0.0f,
               -sin(rot_angle), cos(rot_angle), 0.0f,
               0, 0, 1.0f;

    jacobian = (1.0f/wheel_radius) * jacobian * rot_mat;

    return jacobian;
}
