#ifndef MOBILE_ROBOT_COMMS_OMNIWHEEL_BASE_H
#define MOBILE_ROBOT_COMMS_OMNIWHEEL_BASE_H

#include <Eigen/Core>

#include "kinematics/pose2d.h"
#include "kinematics/differential_drive.h"

class omniwheel_base {
public:
    static constexpr float DEFAULT_WHEEL_RADIUS = 0.065f;
    //static constexpr float DEFAULT_AXIS_LENGTH = 0.25917f;
    static constexpr float DEFAULT_AXIS_LENGTH = 0.335;
    static constexpr float DEFAULT_WHEEL_ANGLE = M_PI / 4.0f;
    static constexpr float DEFAULT_TOP_LEFT_WHEEL_OFFSET = 0.0f;
    static constexpr float DEFAULT_BOTTOM_LEFT_WHEEL_OFFSET = M_PI/2.0f;
    static constexpr float DEFAULT_BOTTOM_RIGHT_WHEEL_OFFSET = M_PI;
    static constexpr float DEFAULT_TOP_RIGHT_WHEEL_OFFSET = 3.0f * M_PI / 2.0f;
public:
    using twist2d = differential_drive::twist2d;
    using pose_with_twist = differential_drive::pose_with_twist;
public:
    struct parameters {
        float wheel_radius {DEFAULT_WHEEL_RADIUS};
        float axis_length {DEFAULT_AXIS_LENGTH};
        float wheel_angle {DEFAULT_WHEEL_ANGLE};
        float top_left_wheel_offset {DEFAULT_TOP_LEFT_WHEEL_OFFSET};
        float bottom_left_wheel_offset {DEFAULT_BOTTOM_LEFT_WHEEL_OFFSET};
        float bottom_right_wheel_offset {DEFAULT_BOTTOM_RIGHT_WHEEL_OFFSET};
        float top_right_wheel_offset {DEFAULT_TOP_RIGHT_WHEEL_OFFSET};
    };

    struct wheel_velocities {
        float top_left_omega {};
        float bottom_left_omega {};
        float bottom_right_omega {};
        float top_right_omega {};
    };

    static pose_with_twist forward_kinematics(const pose2d& pose, const parameters& parameters, const wheel_velocities& velocities, float dt);
    static Eigen::MatrixXf get_local_jacobian(const parameters& parameters);
private:
    omniwheel_base() = default;
};

#endif //MOBILE_ROBOT_COMMS_OMNIWHEEL_BASE_H
