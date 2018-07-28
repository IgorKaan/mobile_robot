#include "base_controller.h"


base_controller::base_controller(std::string rpm_topic, std::string vel_topic)
    : n("base_controller")
{
    n.param<float>("axis_length", m_robot_params.axis_length, 0.30f);
    n.param<float>("wheel_radius", m_robot_params.wheel_radius, 0.10f);
    n.param<float>("ticks_rev", m_robot_params.ticks_rev, 60);
    n.param<float>("max_linear_velocity", m_max_lin_vel, 0.5f);
    n.param<float>("max_angular_velocity", m_max_ang_vel, 2.5f);

    sub = n.subscribe(vel_topic, 32, &base_controller::twist_cb, this);
    pub = n.advertise<std_msgs::Int16MultiArray>(rpm_topic, 32);

    ROS_INFO("base_controller params: L: %f, R: %f, T/rev: %f",
             m_robot_params.axis_length,
             m_robot_params.wheel_radius,
             m_robot_params.ticks_rev
    );

    m_last_cmd_time = ros::Time::now();
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

    m_lin_vel = vx;
    m_ang_vel = az;

    m_last_cmd_time = ros::Time::now();
}

void base_controller::update()
{
    float cb_dt = (ros::Time::now() - m_last_cmd_time).toSec();

    if (cb_dt > 0.5f) {
        m_lin_vel = 0.0f;
        m_ang_vel = 0.0f;
    }

    float vx = m_lin_vel;
    float az = m_ang_vel;

    std_msgs::Int16MultiArray arr_msg;

    arr_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    arr_msg.layout.dim[0].label = "rpm";
    arr_msg.layout.dim[0].size = 2;
    arr_msg.layout.dim[0].stride = 2;

    float L = m_robot_params.axis_length;
    float R = m_robot_params.wheel_radius;

    if (vx > m_max_lin_vel) {
        vx = m_max_lin_vel;
    } else if (vx < -1.0f * m_max_lin_vel) {
        vx = -1.0f * m_max_lin_vel;
    }

    if (az > m_max_ang_vel) {
        az = m_max_ang_vel;
    } else if (az < -1.0f * m_max_ang_vel) {
        az = -1.0f * m_max_ang_vel;
    }

    float left_omega = (2*vx - az * L) / (2.0f * R);
    float right_omega = (2*vx + az * L) / (2.0f * R);

    int left_rpm = left_omega * (60.0f / (2.0f * M_PI));
    int right_rpm = right_omega * (60.0f / (2.0f * M_PI));

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

    ROS_INFO("%f %f %f %f %d %d", vx, az, left_omega, right_omega, left_rpm, right_rpm);

    arr_msg.data.push_back(left_rpm);
    arr_msg.data.push_back(right_rpm);

    pub.publish(arr_msg);
}
