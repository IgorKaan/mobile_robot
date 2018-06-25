#include <ros/ros.h>

#include "kinematics/differential_drive.h"
#include "odometry_publisher.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry_publisher_node");

    float R = 0.06;
    float L = 0.28;

    differential_drive::parameters params;
    params.axis_length = L;
    params.wheel_radius = R;

    odometry_publisher odo_pub("cmd_rpm", "odom", params);

    ros::Rate rate(100);

    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}