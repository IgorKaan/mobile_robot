#include "base_controller.h"


base_controller::base_controller(std::string rpm_topic, std::string vel_topic, differential_drive::parameters robot_params)
        : m_robot_params(robot_params)
{
    sub = n.subscribe(vel_topic, 16, &base_controller::twist_cb, this);
    pub = n.advertise<std_msgs::Int16MultiArray>(rpm_topic, 100);
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

    float L = m_robot_params.axis_length;
    float R = m_robot_params.wheel_radius;

    std::cout << vx << ", " << az << std::endl;

    int right_rpm = (2*vx + az * L) / (2.0f * R);
    int left_rpm = (2*vx - az * L) / (2.0f * R);

    if (left_rpm > 30) {
        left_rpm = 30;
    } else if (left_rpm < -30) {
        left_rpm = -30;
    }

    if (right_rpm > 30) {
        right_rpm = 30;
    } else if (right_rpm < -30) {
        right_rpm = -30;
    }

    arr_msg.data.push_back(left_rpm);
    arr_msg.data.push_back(right_rpm);

    pub.publish(arr_msg);
}