#ifndef MOBILE_ROBOT_COMMS_DIFFERENTIAL_DRIVE_H
#define MOBILE_ROBOT_COMMS_DIFFERENTIAL_DRIVE_H



#include "kinematics/pose2d.h"

// use SI units pls
class differential_drive {
public:
    struct parameters {
        float wheel_radius;
        float axis_length;
        float ticks_rev;
    };

    struct wheel_vels {
        float left_omega;
        float right_omega;
    };

    struct twist2d {
        float vx;
        float vy;
        float omega;
    };

    struct pose_with_twist {
        pose2d pose;
        twist2d twist;
    };

    static pose_with_twist forward_kinematics(pose2d pose, parameters params, wheel_vels vels);
private:
    differential_drive() = default;
};

#endif //MOBILE_ROBOT_COMMS_DIFFERENTIAL_DRIVE_H
