#include <iostream>
#include <cstdlib>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16MultiArray.h>

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

}

int main(int argc, char** argv)
{
    try {
        ros::init(argc, argv, "test_node_listener");
        ros::NodeHandle n;
        ros::Subscriber sub = n.subscribe("cmd_vel", 16, twist_cb);
        ros::Publisher pub = n.advertise<std_msgs::Int16MultiArray>("cmd_rpm", 100);

        ros::Rate rate(30);

        while (ros::ok()) {
            std_msgs::Int16MultiArray arr_msg;

            arr_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
            arr_msg.layout.dim[0].label = "rpm";
            arr_msg.layout.dim[0].size = 2;
            arr_msg.layout.dim[0].stride = 2;
            if (argc > 1) {

                arr_msg.data.push_back(strtol(argv[1], nullptr, 10));
                arr_msg.data.push_back(strtol(argv[2], nullptr, 10));
            } else {
                arr_msg.data.push_back(0);
                arr_msg.data.push_back(0);
            }

            pub.publish(arr_msg);

            ros::spinOnce();
            rate.sleep();
        }
    } catch (std::exception& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }

    return 0;
}

