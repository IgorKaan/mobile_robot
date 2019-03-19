#include "odometry_publisher.h"

odometry_publisher::odometry_publisher(std::string rpm_topic, std::string odom_topic)
        : n("odometry_publisher"), m_pose(0.0f, 0.0f, 0.0f)
{
    n.param<float>("axis_length", m_robot_params.axis_length, 0.30f);
    n.param<float>("wheel_radius", m_robot_params.wheel_radius, 0.10f);
    n.param<float>("ticks_rev", m_robot_params.ticks_rev, 60);
    n.param<float>("lwheel_alpha", m_robot_params.lwheel_alpha, 0.5);
    n.param<float>("rwheel_alpha", m_robot_params.rwheel_alpha, 0.5);

    left_sub = n.subscribe("/lwheel", 10, &odometry_publisher::left_cb, this);
    right_sub = n.subscribe("/rwheel", 10, &odometry_publisher::right_cb, this);
    odom_pub = n.advertise<nav_msgs::Odometry>(odom_topic, 30);

    m_wheel_vels.left_omega = 0.0f;
    m_wheel_vels.right_omega = 0.0f;

    ROS_INFO("odometry_publisher params: L: %f, R: %f, T/rev: %f",
             m_robot_params.axis_length,
             m_robot_params.wheel_radius,
             m_robot_params.ticks_rev
    );

    m_last_odom_msg.header.stamp = ros::Time::now();

    m_prev_time = ros::Time::now();
    m_last_left = m_last_right = ros::Time::now();
}

void odometry_publisher::left_cb(const std_msgs::Int16::ConstPtr &left_msg)
{
    //float left_rpm = (m_left_rpm_old + left_msg->data) / 2.0f;
    float left_rpm = differential_drive::lowpass_filter(m_robot_params.lwheel_alpha, m_left_rpm_old, left_msg->data);
    m_left_rpm_old = left_rpm;

    m_wheel_vels.left_omega = ((2.0f*M_PI) / 60.0f) * left_rpm;
    m_last_left = ros::Time::now();
}

void odometry_publisher::right_cb(const std_msgs::Int16::ConstPtr &right_msg)
{
    //float right_rpm = (m_right_rpm_old + right_msg->data) / 2.0f;
    float right_rpm = differential_drive::lowpass_filter(m_robot_params.rwheel_alpha, m_right_rpm_old, right_msg->data);
    m_right_rpm_old = right_rpm;

    m_wheel_vels.right_omega = ((2.0f*M_PI) / 60.0f) * right_rpm;
    m_last_right = ros::Time::now();
}

nav_msgs::Odometry odometry_publisher::get_last_odom_msg() const
{
    return m_last_odom_msg;
}

void odometry_publisher::update()
{
    ros::Time current_time = ros::Time::now();
    float dt = (current_time - m_prev_time).toSec();
    m_prev_time = current_time;

    float last_left_dt = (current_time - m_last_left).toSec();
    float last_right_dt = (current_time - m_last_right).toSec();

    if (last_left_dt > 0.5f) {
        m_wheel_vels.left_omega = 0.0f;
    }

    if (last_right_dt > 0.5f) {
        m_wheel_vels.right_omega = 0.0f;
    }

    pose2d new_pose;
    differential_drive::twist2d twist;

    differential_drive::pose_with_twist pose_twist = differential_drive::forward_kinematics(m_pose, m_robot_params,
                                                                                            m_wheel_vels, dt);
    new_pose = pose_twist.pose;
    twist = pose_twist.twist;
    ROS_INFO("%f %f %f %f %f %f %f", m_wheel_vels.left_omega, m_wheel_vels.right_omega,
             new_pose.get_x(), new_pose.get_y(), new_pose.get_theta(), dt);

    m_pose = new_pose;

    float yaw = m_pose.get_theta();
    geometry_msgs::Quaternion orient_quat = tf::createQuaternionMsgFromYaw(yaw);

    geometry_msgs::TransformStamped odom_transform;
    odom_transform.header.stamp = current_time;
    odom_transform.header.frame_id = "odom";
    odom_transform.child_frame_id = "base_link";

    odom_transform.transform.translation.x = m_pose.get_x();
    odom_transform.transform.translation.y = m_pose.get_y();
    odom_transform.transform.rotation = orient_quat;

    //odom_broadcaster.sendTransform(odom_transform);

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    // Pose in frame_id
    odom_msg.pose.pose.position.x = m_pose.get_x();
    odom_msg.pose.pose.position.y = m_pose.get_y();
    odom_msg.pose.pose.orientation = orient_quat;

    // Twist in child_frame_id
    odom_msg.twist.twist.linear.x = twist.vx;
    odom_msg.twist.twist.linear.y = 0.0f;
    odom_msg.twist.twist.angular.z = twist.omega;

    odom_msg.pose.covariance = {
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    };

    odom_msg.twist.covariance = {
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 999, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 999, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 999, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    };

    m_last_odom_msg = odom_msg;

    odom_pub.publish(odom_msg);
}
