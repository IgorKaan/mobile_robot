#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

void twist_cb(const geometry_msgs::Twist::ConstPtr& twist_msg)
{
    float vx, vy, vz;
    vx = twist_msg->linear.x;
    vy = twist_msg->linear.y;
    vz = twist_msg->linear.z;

    float ax, ay, az;
    ax = twist_msg->angular.x;
    ay = twist_msg->angular.y;
    az = twist_msg->angular.z;

    ROS_INFO("TWISTING");
    ROS_INFO("Linear: [%f] [%f] [%f]", vx, vy, vz);
    ROS_INFO("Angular: [%f] [%f] [%f]", ax, ay, az);
}

int main(int argc, char** argv)
{
    try {
        ros::init(argc, argv, "test_node_listener");
        ros::NodeHandle n;
        ros::Subscriber sub = n.subscribe("turtle1/cmd_vel", 10, twist_cb);

        ros::Rate rate(30);

        ROS_INFO("Privyet, I am test echo node");
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

