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
#include "kinematics/omniwheel_base.h"

class odometry_publisher {
public:
    static const unsigned int WHEEL_COUNT = 4;

    enum wheel_id {
        TOP_LEFT = 0,
        BOTTOM_LEFT = 1,
        BOTTOM_RIGHT = 2,
        TOP_RIGHT = 3
    };
public:
    odometry_publisher(std::string rpm_topic, std::string odom_topic);

    void top_left_cb(const std_msgs::Int8::ConstPtr& rpm_msg);
    void bottom_left_cb(const std_msgs::Int8::ConstPtr& rpm_msg);
    void bottom_right_cb(const std_msgs::Int8::ConstPtr& rpm_msg);
    void top_right_cb(const std_msgs::Int8::ConstPtr& rpm_msg);

    nav_msgs::Odometry get_last_odom_msg() const;

    void update();

    void get_omega_from_rpm(int wheel_rpm, int id);

private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_wheel_subs[WHEEL_COUNT];
    ros::Publisher m_odom_pub;
    tf::TransformBroadcaster m_odom_broadcaster;

    nav_msgs::Odometry m_last_odom_msg;

    float m_old_wheel_rpm[WHEEL_COUNT];

    pose2d m_pose;

    omniwheel_base::parameters m_robot_params;
    float m_wheel_velocity[WHEEL_COUNT];

    ros::Time m_prev_time;
    ros::Time m_last_wheel_time[WHEEL_COUNT];
};

#endif //MOBILE_ROBOT_COMMS_ODOMETRY_PUBLISHER_H
