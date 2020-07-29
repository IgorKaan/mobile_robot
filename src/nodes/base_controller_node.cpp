#include <iostream>
#include <cstdlib>

#include <ros/ros.h>

#include "base_controller.h"

int main(int argc, char** argv)
{
    try {
        ros::init(argc, argv, "base_controller_node");

        base_controller controller("/cmd_vel");

        ros::Rate rate(30.0f);
        while (ros::ok()) {
            controller.update();
            ros::spinOnce();
            rate.sleep();
        }
    } catch (std::exception& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }

    return 0;
}

