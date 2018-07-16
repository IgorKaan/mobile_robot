#include "odometry_publisher.h"

odometry_publisher::odometry_publisher(std::string rpm_topic, std::string odom_topic)
        : n("odometry_publisher"), m_pose(0.0f, 0.0f, 0.0f),
          m_left_rpm_old(0.0f), m_right_rpm_old(0.0f)
{
    n.param<float>("axis_length", m_robot_params.axis_length, 0.30f);
    n.param<float>("wheel_radius", m_robot_params.wheel_radius, 0.10f);
    n.param<float>("ticks_rev", m_robot_params.ticks_rev, 60);

    rpm_sub = n.subscribe(rpm_topic, 10, &odometry_publisher::odometry_cb, this);
    odom_pub = n.advertise<nav_msgs::Odometry>(odom_topic, 10);

    ROS_INFO("odometry_publisher params: L: %f, R: %f, T/rev: %f",
             m_robot_params.axis_length,
             m_robot_params.wheel_radius,
             m_robot_params.ticks_rev
    );
}

void odometry_publisher::odometry_cb(const std_msgs::Int16MultiArray::ConstPtr &rpm_msg)
{
    ros::Time current_time = ros::Time::now();
    float dt = (current_time - m_prev_time).toSec();
    m_prev_time = current_time;

    pose2d new_pose;
    differential_drive::twist2d twist;

    float left_rpm = (m_left_rpm_old + rpm_msg->data[0]) / 2.0f;
    float right_rpm = (m_right_rpm_old + rpm_msg->data[1]) / 2.0f;
    m_left_rpm_old = rpm_msg->data[0];
    m_right_rpm_old = rpm_msg->data[1];

    differential_drive::wheel_vels vels;
    vels.left_omega = (M_2_PI / 60.0f) * left_rpm;
    vels.right_omega = (M_2_PI / 60.0f) * right_rpm;

    differential_drive::pose_with_twist pose_twist = differential_drive::forward_kinematics(m_pose, m_robot_params, vels, dt);
    new_pose = pose_twist.pose;
    twist = pose_twist.twist;
    ROS_INFO("%f %f %f %f %f %f %f", left_rpm, right_rpm,
             new_pose.get_x(), new_pose.get_y(), new_pose.get_theta(), dt, (1.0/dt));

    m_pose = new_pose;

    float rate = 10.0f;
    float yaw = m_pose.get_theta() * rate;
    geometry_msgs::Quaternion orient_quat = tf::createQuaternionMsgFromYaw(yaw);

    geometry_msgs::TransformStamped odom_transform;
    odom_transform.header.stamp = current_time;
    odom_transform.header.frame_id = "odom";
    odom_transform.child_frame_id = "base_link";

    odom_transform.transform.translation.x = new_pose.get_x() * rate;
    odom_transform.transform.translation.y = new_pose.get_y() * rate;
    odom_transform.transform.rotation = orient_quat;

    odom_broadcaster.sendTransform(odom_transform);

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    // Pose in frame_id
    odom_msg.pose.pose.position.x = new_pose.get_x() * rate;
    odom_msg.pose.pose.position.y = new_pose.get_y() * rate;
    odom_msg.pose.pose.orientation = orient_quat;

    // Twist in child_frame_id
    odom_msg.twist.twist.linear.x = twist.vx;
    odom_msg.twist.twist.linear.y = 0.0f;
    odom_msg.twist.twist.angular.z = twist.omega;

    odom_pub.publish(odom_msg);

}
