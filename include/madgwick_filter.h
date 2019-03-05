#pragma once

#include <madgwick_filter_library.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Imu.h>

namespace platform_imu {

class madgwick_filter {
    ros::NodeHandle n;
    geometry_msgs::Point point;
    madgwick_filter_library madg;
    ros::Subscriber sub;
    ros::Publisher pub;

    void callback(const sensor_msgs::Imu& pos);

public:
    madgwick_filter();
};
}