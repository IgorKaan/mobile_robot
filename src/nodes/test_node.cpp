#include <iostream>
#include <cstdlib>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16MultiArray.h>



class Pubsub {
public:
    Pubsub(std::string pub_topic, std::string sub_topic)
    {
        sub = n.subscribe("cmd_vel", 16, &Pubsub::twist_cb, this);
        pub = n.advertise<std_msgs::Int16MultiArray>("cmd_rpm", 100);
    }

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

        std_msgs::Int16MultiArray arr_msg;

        arr_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        arr_msg.layout.dim[0].label = "rpm";
        arr_msg.layout.dim[0].size = 2;
        arr_msg.layout.dim[0].stride = 2;

        float v_mag = sqrt(vx*vx + vy*vy + vz*vz);
        float L = 0.28;
        float R = 0.06;

        std::cout << v_mag << ", " << az << std::endl;

        v_mag /=  10.0f;

        int right_rpm = (2*v_mag + az * L) / (2 * R);
        int left_rpm = (2*v_mag - az * L) / (2 * R);

        if (left_rpm > 30) {
            left_rpm = 30;
        }

        if (right_rpm > 30) {
            right_rpm = 30;
        }

        arr_msg.data.push_back(left_rpm);
        arr_msg.data.push_back(right_rpm);

        pub.publish(arr_msg);
    }
private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;
};

int main(int argc, char** argv)
{
    try {
        ros::init(argc, argv, "test_node_listener");

        Pubsub ps("cmd_rpm", "cmd_vel");

        ros::Rate rate(30);
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

