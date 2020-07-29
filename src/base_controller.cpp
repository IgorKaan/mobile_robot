#include "base_controller.h"

base_controller::base_controller(std::string vel_topic)
    : n("base_controller")
{
    n.param<float>("max_linear_velocity", m_max_lin_vel, 0.5f);
    n.param<float>("max_angular_velocity", m_max_ang_vel, 2.0f);

    sub = n.subscribe(vel_topic, 32, &base_controller::twist_cb, this);

    top_left_pub = n.advertise<std_msgs::Int8>("/rpm_top_left_pub", 10);
    bottom_left_pub = n.advertise<std_msgs::Int8>("/rpm_bottom_left_pub", 10);
    bottom_right_pub = n.advertise<std_msgs::Int8>("/rpm_bottom_right_pub", 10);
    top_right_pub = n.advertise<std_msgs::Int8>("/rpm_top_right_pub", 10);

    ROS_INFO("Initialized base_controller for omniwheel_drive");

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

    m_lin_vel_x = vx;
    m_lin_vel_y = vy;
    m_ang_vel = az;

    m_last_cmd_time = ros::Time::now();
}

void base_controller::update()
{
    float cb_dt = (ros::Time::now() - m_last_cmd_time).toSec();

    if (cb_dt > 0.5f) {
        m_lin_vel_x = 0.0f;
        m_lin_vel_y = 0.0f;
        m_ang_vel = 0.0f;
    }

    float vx = m_lin_vel_x;
    float vy = m_lin_vel_y;
    float az = m_ang_vel;

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

    Eigen::MatrixXf jacobian = omniwheel_base::get_local_jacobian(m_robot_params);
    Eigen::MatrixXf twist_vec = Eigen::MatrixXf(3, 1);
    twist_vec << vx, vy, az;

    Eigen::MatrixXf wheel_vel_vector = jacobian * twist_vec;
    float top_left_omega = wheel_vel_vector(0, 0);
    float bottom_left_omega = wheel_vel_vector(1, 0);
    float bottom_right_omega = wheel_vel_vector(2, 0);
    float top_right_omega = wheel_vel_vector(3, 0);

    int top_left_rpm = top_left_omega * (60.0f / (2.0f * M_PI));
    int bottom_left_rpm = bottom_left_omega * (60.0f / (2.0f * M_PI));
    int bottom_right_rpm = bottom_right_omega * (60.0f / (2.0f * M_PI));
    int top_right_rpm = top_right_omega * (60.0f / (2.0f * M_PI));

    //ROS_INFO("%f %f %f %f %d %d", vx, az, left_omega, right_omega, left_rpm, right_rpm);

    std_msgs::Int8 top_left_msg, bottom_left_msg;
    std_msgs::Int8 bottom_right_msg, top_right_msg;

    top_left_msg.data = top_left_rpm;
    bottom_left_msg.data = bottom_left_rpm;
    bottom_right_msg.data = bottom_right_rpm;
    top_right_msg.data = top_right_rpm;

    top_left_pub.publish(top_left_msg);
    bottom_left_pub.publish(bottom_left_msg);
    bottom_right_pub.publish(bottom_right_msg);
    top_right_pub.publish(top_right_msg);
}
