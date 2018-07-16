#include <iostream>
#include <cstdlib>

#include <ros/ros.h>

#include "base_controller.h"

int main(int argc, char** argv)
{
    try {
        ros::init(argc, argv, "base_controller_node");

        float L = 0.28f;
        float R = 0.06f;

        differential_drive::parameters params;
        params.axis_length = L;
        params.wheel_radius = R;

        base_controller controller("/cmd_rpm", "/cmd_vel");

        ros::Rate rate(60.0f);
        ROS_INFO("Sub: cmd_vel, pub: cmd_rpm");
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    } catch (std::exception& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }

    return 0;
}

