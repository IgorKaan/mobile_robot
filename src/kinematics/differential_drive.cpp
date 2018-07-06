#include <iostream>
#include "kinematics/differential_drive.h"

differential_drive::pose_with_twist differential_drive::forward_kinematics(
        pose2d pose, differential_drive::parameters params,
        float left_omega, float right_omega, float dt)
{
    pose_with_twist result;

    pose2d new_pose;
    twist2d twist;

    float x = pose.get_x(), y = pose.get_y(), theta = pose.get_theta();

    // wheel velocities
    // omega is in rad/s
    float left_vel = left_omega * params.wheel_radius;
    float right_vel = right_omega * params.wheel_radius;

    // Velocity in robot frame (x axis - forward)
    // robot_frame_vel_x = velocity_base
    // robot_frame_vel_y = 0
    float velocity_base = 0.5f * (right_vel + left_vel);

    if (std::fabs(left_vel - right_vel) < 0.001f) {
        new_pose.set_x(x + std::cos(theta)*velocity_base * dt);
        new_pose.set_y(y + std::sin(theta)*velocity_base * dt);
        new_pose.set_theta(theta);
	
        // Forward linear motion
        twist.vx = velocity_base;
	twist.vy = 0.0f;
        twist.omega = 0.0f;
    } else {
        // R from ICC to pose
        float curve_R = (params.axis_length / 2.0f) * (left_vel + right_vel) / (right_vel - left_vel);
        // angular velocity about ICC
        float omega = (right_vel - left_vel) / params.axis_length;
        float dw = omega * dt;

        float ICC_x = x - curve_R * std::sin(theta);
        float ICC_y = y - curve_R * std::cos(theta);

        float new_x = std::cos(dw) * (x - ICC_x) - std::sin(dw) * (y - ICC_y) + ICC_x;
        float new_y = std::sin(dw) * (x - ICC_x) + std::cos(dw) * (y - ICC_y) + ICC_y;
        
        new_pose.set_x(new_x);
        new_pose.set_y(new_y);
        new_pose.set_theta(theta);
        // Angles are normalized
        new_pose.set_theta(new_pose.get_theta() + dw);

        // velocity = free vector
        // Convert to velocity in world frame ( :thinking: )
        twist.vx = velocity_base;
        twist.vy = 0.0f;
        twist.omega = omega;
    }

    result.pose = new_pose;
    result.twist = twist;

    return result;
}
