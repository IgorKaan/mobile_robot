#include "base_controller.h"


base_controller::base_controller(std::string pub_topic, std::string sub_topic)
{
    sub = n.subscribe("cmd_vel", 16, &base_controller::twist_cb, this);
    pub = n.advertise<std_msgs::Int16MultiArray>("cmd_rpm", 100);
}

void base_controller::twist_cb(const geometry_msgs::Twist::ConstPtr& twist_msg)
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
    az /= 1.5f;

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