#include "base_controller.h"


base_controller::base_controller(std::string rpm_topic, std::string vel_topic)
    : n("base_controller")
{
    n.param<float>("axis_length", m_robot_params.axis_length, 0.30f);
    n.param<float>("wheel_radius", m_robot_params.wheel_radius, 0.10f);
    n.param<float>("ticks_rev", m_robot_params.ticks_rev, 60);

    sub = n.subscribe(vel_topic, 10, &base_controller::twist_cb, this);
    pub = n.advertise<std_msgs::Int16MultiArray>(rpm_topic, 10);

    ROS_INFO("base_controller params: L: %f, R: %f, T/rev: %f",
             m_robot_params.axis_length,
             m_robot_params.wheel_radius,
             m_robot_params.ticks_rev
    );
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

    vx /= 10.0f;

    if (vx > 1.5f) {
        vx = 1.5f;
    } else if (vx < -1.5f) {
        vx = -1.5f;
    }

    if (az > 10.0f) {
        az = 10.0f;
    } else if (az < -10.0f) {
        az = -10.0f;
    }

    ROS_INFO("%f %f", vx, az);

    float left_vel = (2*vx - az * L) / (2.0f * R);
    float right_vel = (2*vx + az * L) / (2.0f * R);
    
    float left_omega = left_vel / R;
    float right_omega = right_vel / R;

    //vels.left_omega = (M_2_PI / 60.0f) * left_rpm;
    //vels.right_omega = (M_2_PI / 60.0f) * right_rpm;

    int left_rpm = left_omega * (60.0f / M_2_PI);
    int right_rpm = right_omega * (60.0f / M_2_PI);

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
