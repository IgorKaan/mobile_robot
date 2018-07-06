#include "odometry_publisher.h"

odometry_publisher::odometry_publisher(std::string rpm_topic, std::string odom_topic, differential_drive::parameters robot_params)
        : m_pose(0.0f, 0.0f, 0.0f), m_robot_params(robot_params)
{
    rpm_sub = n.subscribe(rpm_topic, 16, &odometry_publisher::odometry_cb, this);
    odom_pub = n.advertise<nav_msgs::Odometry>(odom_topic, 100);
}

void odometry_publisher::odometry_cb(const std_msgs::Int16MultiArray::ConstPtr &rpm_msg)
{
    ros::Time current_time = ros::Time::now();
    float dt = (current_time - prev_time).toSec();

    int left_rpm = rpm_msg->data[0];
    int right_rpm = rpm_msg->data[1];

    float left_omega = (M_2_PI / 60) * left_rpm;
    float right_omega = (M_2_PI / 60) * right_rpm;

    differential_drive::pose_with_twist pose_twist = differential_drive::forward_kinematics(m_pose, m_robot_params,
                                                                                          left_omega, right_omega, dt);

    pose2d new_pose = pose_twist.pose;
    differential_drive::twist2d twist = pose_twist.twist;

    ROS_INFO("%d %d %f %f", left_rpm, right_rpm, left_omega, right_omega);

    m_pose = new_pose;

    geometry_msgs::Quaternion orient_quat = tf::createQuaternionMsgFromYaw(new_pose.get_theta());

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

    odom_msg.pose.pose.position.x = new_pose.get_x();
    odom_msg.pose.pose.position.y = new_pose.get_y();
    odom_msg.pose.pose.orientation = orient_quat;

    odom_msg.twist.twist.linear.x = twist.vx;
    odom_msg.twist.twist.linear.y = 0.0f;
    odom_msg.twist.twist.angular.z = twist.omega;

    odom_pub.publish(odom_msg);

    prev_time = current_time;
}
