#include <iostream>
#include <cstdlib>

#include <ros/ros.h>

#include "sonar_publisher.h"

int main(int argc, char** argv)
{
    try {
        ros::init(argc, argv, "sonar_publisher_node");

        sonar_publisher sonar_pub(0.1f, 0.1, 1.0f);

        ros::Rate rate(30.0f);
        while (ros::ok()) {
            sonar_pub.update();
            ros::spinOnce();
            rate.sleep();
        }
    } catch (std::exception& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }

    return 0;
}

