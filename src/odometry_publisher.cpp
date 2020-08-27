#include "odometry_publisher.h"

#include "kinematics/omniwheel_base.h"

odometry_publisher::odometry_publisher(std::string rpm_topic, std::string odom_topic)
        : m_nh("odometry_publisher"), m_pose(0.0f, 0.0f, 0.0f)
{
    m_nh.param<float>("axis_length", m_robot_params.axis_length, omniwheel_base::DEFAULT_AXIS_LENGTH);
    m_nh.param<float>("wheel_radius", m_robot_params.wheel_radius, omniwheel_base::DEFAULT_WHEEL_RADIUS);

    m_wheel_subs[wheel_id::TOP_LEFT] = m_nh.subscribe("/rpm_top_left", 10, &odometry_publisher::top_left_cb, this);
    m_wheel_subs[wheel_id::BOTTOM_LEFT] = m_nh.subscribe("/rpm_bottom_left", 10, &odometry_publisher::bottom_left_cb, this);
    m_wheel_subs[wheel_id::BOTTOM_RIGHT] = m_nh.subscribe("/rpm_bottom_right", 10, &odometry_publisher::bottom_right_cb, this);
    m_wheel_subs[wheel_id::TOP_RIGHT] = m_nh.subscribe("/rpm_top_right", 10, &odometry_publisher::top_right_cb, this);

    m_odom_pub = m_nh.advertise<nav_msgs::Odometry>(odom_topic, 30);

    ROS_INFO("odometry_publisher started");

    m_last_odom_msg.header.stamp = ros::Time::now();

    m_prev_time = ros::Time::now();

    for (int wid = 0; wid < WHEEL_COUNT; wid++) {
        m_wheel_velocity[wid] = 0.0f;
        m_last_wheel_time[wid] = ros::Time::now();
    }
}

void odometry_publisher::top_left_cb(const std_msgs::Int8::ConstPtr &rpm_msg)
{
    get_omega_from_rpm(rpm_msg->data, wheel_id::TOP_LEFT);
}

void odometry_publisher::bottom_left_cb(const std_msgs::Int8::ConstPtr &rpm_msg) {
    get_omega_from_rpm(rpm_msg->data, wheel_id::BOTTOM_LEFT);
}

void odometry_publisher::bottom_right_cb(const std_msgs::Int8::ConstPtr &rpm_msg) {
    get_omega_from_rpm(rpm_msg->data, wheel_id::BOTTOM_RIGHT);
}

void odometry_publisher::top_right_cb(const std_msgs::Int8::ConstPtr &rpm_msg)
{
    get_omega_from_rpm(rpm_msg->data, wheel_id::TOP_RIGHT);
}

void odometry_publisher::get_omega_from_rpm(int wheel_rpm, int id) {
    m_old_wheel_rpm[id] = wheel_rpm;
    m_wheel_velocity[id] = ((2.0f*M_PI) / 60.0f) * wheel_rpm;
    m_last_wheel_time[id] = ros::Time::now();
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

    for (int wid = 0; wid < WHEEL_COUNT; wid++) {
        if ((current_time - m_last_wheel_time[wid]).toSec() > 0.5f) {
            m_wheel_velocity[wid] = 0.0f;
        }
    }

    pose2d new_pose;
    differential_drive::twist2d twist;

    /*
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
    odom_msg.pose.pose.position.z = 0.0f;
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
    */
}
