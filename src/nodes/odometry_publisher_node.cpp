#include <ros/ros.h>

#include "kinematics/differential_drive.h"
#include "odometry_publisher.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry_publisher_node");
    odometry_publisher odo_pub("/rpm_data", "/odom");

    ros::Rate rate(60.0f);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
