#include <iostream>
#include <cstdlib>
#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>

class imu_filter_node {
public:
    imu_filter_node(const std::string& gyro_topic, const std::string& accel_topic)
        : m_nh("imu_filter_node")
    {
        m_pub = m_nh.advertise<sensor_msgs::Imu>("/imu_raw", 32);

        m_gyro_sub = m_nh.subscribe<geometry_msgs::Vector3>(gyro_topic, 32, &imu_filter_node::gyro_callback, this);
        m_accel_sub = m_nh.subscribe<geometry_msgs::Vector3>(accel_topic, 32, &imu_filter_node::accel_callback, this);

	m_gyro_msg.x = 0.0f;
	m_gyro_msg.y = 0.0f;
	m_gyro_msg.z = 0.0f;

	m_accel_msg.x = 0.0f;
	m_accel_msg.y = 0.0f;
	m_accel_msg.z = 0.0f;
    }

    void gyro_callback(const geometry_msgs::Vector3 gyro_msg) {
        m_gyro_msg.x = gyro_msg.x;
        //m_gyro_msg.y = -gyro_msg.y;
        //m_gyro_msg.z = -gyro_msg.z;
        m_gyro_msg.y = gyro_msg.y;
        m_gyro_msg.z = gyro_msg.z;
    }

    void accel_callback(const geometry_msgs::Vector3 accel_msg) {
        m_accel_msg.x = accel_msg.x;
        //m_accel_msg.y = -accel_msg.y;
        //m_accel_msg.z = -accel_msg.z;
        m_accel_msg.y = accel_msg.y;
        m_accel_msg.z = accel_msg.z;
    }

    void imu_filter_update()
    {
        sensor_msgs::Imu imu_msg;

        imu_msg.header.frame_id = "base_link";
        imu_msg.header.stamp = ros::Time::now();

	imu_msg.orientation.x = 0.0;
	imu_msg.orientation.y = 0.0;
	imu_msg.orientation.z = 0.0;
	imu_msg.orientation.w = 1.0;
        imu_msg.orientation_covariance.elems[0] = -1;

        imu_msg.angular_velocity = m_gyro_msg;
        imu_msg.angular_velocity_covariance.elems[0] = 0.0005;
        imu_msg.angular_velocity_covariance.elems[4] = 0.0005;
        imu_msg.angular_velocity_covariance.elems[8] = 0.0005;

        imu_msg.linear_acceleration = m_accel_msg;
        imu_msg.linear_acceleration_covariance.elems[0] = 0.0025;
        imu_msg.linear_acceleration_covariance.elems[4] = 0.0025;
        imu_msg.linear_acceleration_covariance.elems[8] = 0.0025;

        m_pub.publish(imu_msg);
    }
private:
    ros::NodeHandle m_nh;
    ros::Publisher m_pub;
    ros::Subscriber m_gyro_sub, m_accel_sub;

    geometry_msgs::Vector3 m_gyro_msg, m_accel_msg;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_filter");

    imu_filter_node filter("/gyro", "/accel");
    ROS_INFO("Starting starting imu filter node");

    ros::Rate rate(100.0f);
    while (ros::ok()) {
        filter.imu_filter_update();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

