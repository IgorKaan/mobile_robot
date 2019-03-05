#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>

#include "madgwick_filter.h"

using namespace platform_imu;

madgwick_filter::madgwick_filter() {
    ros::Rate loop_rate(1000);
    sub = n.subscribe("imu", 1000, &madgwick_filter::callback, this);
    pub = n.advertise<sensor_msgs::Imu>("imu_data", 1000);
    m_last_callback = ros::Time::now();
}

void madgwick_filter::callback(const sensor_msgs::Imu& pos) {
    float delta = (ros::Time::now() - m_last_callback).toSec();

    geometry_msgs::Point angles;
    madgwick_filter_library madg;
    madgwick_filter_library::quaternion quaternion;
    quaternion.w = 1.0f; quaternion.x = 0.0f; quaternion.y = 0.0f; quaternion.z = 0.0f;
    madg.set_orientation(quaternion);
    madg.set_drift_bias_gain(0.0f);
    madg.set_algorithm_gain(0.1f);
    madg.madgwick_update(pos.angular_velocity.x, pos.angular_velocity.y, pos.angular_velocity.z,
            pos.linear_acceleration.x, pos.linear_acceleration.y, pos.linear_acceleration.z, delta);
    angles = madg.get_angles_rad_msg();

    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(angles.x, angles.y, angles.z);
    sensor_msgs::Imu pos_filtered = pos;

    pos_filtered.header.frame_id = "base_link";
    pos_filtered.header.stamp = ros::Time::now();

    pos_filtered.orientation.x = quat.x;
    pos_filtered.orientation.y = quat.y;
    pos_filtered.orientation.z = quat.z;
    pos_filtered.orientation.w = quat.w;
    pos_filtered.orientation_covariance[0] = 0.1;
    pos_filtered.orientation_covariance[4] = 0.1;
    pos_filtered.orientation_covariance[8] = 0.1;

    pub.publish(pos_filtered);

    m_last_callback = ros::Time::now();
}