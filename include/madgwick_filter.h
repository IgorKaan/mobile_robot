#pragma once

#include <madgwick_filter_library.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Imu.h>

#include <nav_msgs/Odometry.h>

namespace platform_imu {

class madgwick_filter {
public:
    madgwick_filter();

    ros::NodeHandle get_node_handle() const;

    void odom_imu_sync(const nav_msgs::OdometryConstPtr& odom_msg, const sensor_msgs::ImuConstPtr& scan_msg);
private:
    ros::NodeHandle n;
    geometry_msgs::Point point;
    madgwick_filter_library madg;
    ros::Subscriber sub;
    ros::Publisher pub, m_imu_synced_pub, m_odom_synced_pub;
    ros::Time m_last_callback;

    void callback(const sensor_msgs::Imu& pos);
};

}