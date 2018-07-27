#include "odometry_publisher.h"

odometry_publisher::odometry_publisher(std::string rpm_topic, std::string odom_topic)
        : n("odometry_publisher"), m_pose(0.0f, 0.0f, 0.0f)
{
    n.param<float>("axis_length", m_robot_params.axis_length, 0.30f);
    n.param<float>("wheel_radius", m_robot_params.wheel_radius, 0.10f);
    n.param<float>("ticks_rev", m_robot_params.ticks_rev, 60);

    rpm_sub = n.subscribe(rpm_topic, 30, &odometry_publisher::odometry_cb, this);
    odom_pub = n.advertise<nav_msgs::Odometry>(odom_topic, 30);
    m_wheel_vels.left_omega = 0.0f;
    m_wheel_vels.right_omega = 0.0f;

    ROS_INFO("odometry_publisher params: L: %f, R: %f, T/rev: %f",
             m_robot_params.axis_length,
             m_robot_params.wheel_radius,
             m_robot_params.ticks_rev
    );
    
    m_prev_time = ros::Time::now();
    m_last_command_time = ros::Time::now();
}

void odometry_publisher::odometry_cb(const std_msgs::Int16MultiArray::ConstPtr& rpm_msg)
{
    float left_rpm = (m_left_rpm_old + rpm_msg->data[0]) / 2.0f;
    float right_rpm = (m_right_rpm_old + rpm_msg->data[1]) / 2.0f;
    m_left_rpm_old = rpm_msg->data[0];
    m_right_rpm_old = rpm_msg->data[1];

    // TODO: maybe lock?
    m_wheel_vels.left_omega = ((2.0f*M_PI) / 60.0f) * left_rpm;
    m_wheel_vels.right_omega = ((2.0f*M_PI) / 60.0f) * right_rpm;

    m_last_command_time = ros::Time::now();
}

void odometry_publisher::update()
{
    ros::Time current_time = ros::Time::now();
    float dt = (current_time - m_prev_time).toSec();
    m_prev_time = current_time;

    float last_command_dt = (current_time - m_last_command_time).toSec();

    if (last_command_dt > 0.5f) {
        m_wheel_vels.left_omega = 0.0f;
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

    odom_transform.transform.translation.x = new_pose.get_x();
    odom_transform.transform.translation.y = new_pose.get_y();
    odom_transform.transform.rotation = orient_quat;

    odom_broadcaster.sendTransform(odom_transform);

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    // Pose in frame_id
    odom_msg.pose.pose.position.x = new_pose.get_x();
    odom_msg.pose.pose.position.y = new_pose.get_y();
    odom_msg.pose.pose.orientation = orient_quat;

    // Twist in child_frame_id
    odom_msg.twist.twist.linear.x = twist.vx;
    odom_msg.twist.twist.linear.y = 0.0f;
    odom_msg.twist.twist.angular.z = twist.omega;

    odom_pub.publish(odom_msg);
}
