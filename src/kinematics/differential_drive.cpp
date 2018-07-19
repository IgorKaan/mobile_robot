#include <iostream>
#include "kinematics/differential_drive.h"

differential_drive::pose_with_twist
differential_drive::forward_kinematics(pose2d pose, parameters params, wheel_vels vels, float dt)
{
    pose_with_twist result;

    pose2d new_pose;
    twist2d twist;

    float x = pose.get_x(), y = pose.get_y(), theta = pose.get_theta();

    // wheel velocities in m/s
    // omega is in rad/s
    // all velocities are instantaneous
    float left_vel = vels.left_omega * params.wheel_radius;
    float right_vel = vels.right_omega * params.wheel_radius;

    if (fabs(left_vel) < 0.0001f && fabs(right_vel) < 0.0001f) {
        result.pose = pose;

        twist.vx = 0.0f;
        twist.vy = 0.0f;
        twist.omega = 0.0f;
        result.twist = twist;

        return result;
    }

    // Velocity in robot frame (x axis - forward)
    // robot_frame_vel_x = velocity_base
    // robot_frame_vel_y = 0
    float velocity_base = 0.5f * (right_vel + left_vel);

    if (std::fabs(left_vel - right_vel) < 0.00001f) {
        // Forward linear motion
        new_pose.set_x(x + std::cos(theta) * velocity_base);
        new_pose.set_y(y + std::sin(theta) * velocity_base);
        new_pose.set_theta(theta);
	
        twist.vx = velocity_base;
        twist.vy = 0.0f;
        twist.omega = 0.0f;
    } else {
        // R from ICC to pose
        float curve_R = (params.axis_length / 2.0f) * ((left_vel + right_vel) / (right_vel - left_vel));
        // angular velocity about ICC
        float omega = (right_vel - left_vel) / params.axis_length;
        float dtheta = omega;

        float ICC_x = x - curve_R * std::sin(theta);
        float ICC_y = y + curve_R * std::cos(theta);

        float new_x = std::cos(dtheta) * (x - ICC_x) - std::sin(dtheta) * (y - ICC_y) + ICC_x;
        float new_y = std::sin(dtheta) * (x - ICC_x) + std::cos(dtheta) * (y - ICC_y) + ICC_y;
        /*
        float omega = (right_vel - left_vel) * params.axis_length;
        
        new_pose.set_x(x + std::cos(theta) * velocity_base * dt);
        new_pose.set_y(y + std::sin(theta) * velocity_base * dt);
        new_pose.set_theta(theta);
        new_pose.set_theta(new_pose.get_theta() + omega * dt);
        
        */
        
        new_pose.set_x(new_x);
        new_pose.set_y(new_y);
        new_pose.set_theta(theta);
        // Angles are normalized
        new_pose.set_theta(new_pose.get_theta() + dtheta);

        // Twist is in robot frame
        twist.vx = velocity_base;
        twist.vy = 0.0f;
        twist.omega = omega;
    }

    result.pose = new_pose;
    result.twist = twist;

    return result;
}
