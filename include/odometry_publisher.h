#ifndef MOBILE_ROBOT_COMMS_ODOMETRY_PUBLISHER_H
#define MOBILE_ROBOT_COMMS_ODOMETRY_PUBLISHER_H

#include <string>

#include <std_msgs/Int8.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

#include "kinematics/pose2d.h"
#include "kinematics/differential_drive.h"

class odometry_publisher {
public:
    odometry_publisher(std::string rpm_topic, std::string odom_topic);

    void left_cb(const std_msgs::Int8::ConstPtr& left_msg);
    void right_cb(const std_msgs::Int8::ConstPtr& right_msg);

    nav_msgs::Odometry get_last_odom_msg() const;

    void update();
private:
    ros::NodeHandle n;
    ros::Subscriber left_sub, right_sub;
    ros::Publisher odom_pub;
    tf::TransformBroadcaster odom_broadcaster;

    nav_msgs::Odometry m_last_odom_msg;

    float m_left_rpm_old {0.0f};
    float m_right_rpm_old {0.0f};
    pose2d m_pose;
    differential_drive::wheel_vels m_wheel_vels;
    differential_drive::parameters m_robot_params;
    ros::Time m_prev_time, m_last_left, m_last_right;
};

#endif //MOBILE_ROBOT_COMMS_ODOMETRY_PUBLISHER_H
