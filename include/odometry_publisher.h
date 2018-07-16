#ifndef MOBILE_ROBOT_COMMS_ODOMETRY_PUBLISHER_H
#define MOBILE_ROBOT_COMMS_ODOMETRY_PUBLISHER_H

#include <string>

#include <std_msgs/Int16MultiArray.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

#include "kinematics/pose2d.h"
#include "kinematics/differential_drive.h"

class odometry_publisher {
public:
    odometry_publisher(std::string rpm_topic, std::string odom_topic, differential_drive::parameters robot_params);

    void odometry_cb(const std_msgs::Int16MultiArray::ConstPtr& rpm_msg);
private:
    ros::NodeHandle n;
    ros::Subscriber rpm_sub;
    ros::Publisher odom_pub;
    tf::TransformBroadcaster odom_broadcaster;

    float m_left_rpm_old {0.0f};
    float m_right_rpm_old {0.0f};
    pose2d m_pose;
    differential_drive::parameters m_robot_params;
    ros::Time m_prev_time;
};

#endif //MOBILE_ROBOT_COMMS_ODOMETRY_PUBLISHER_H
