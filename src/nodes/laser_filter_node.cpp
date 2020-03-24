#include <iostream>
#include <cstdlib>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class laser_filter {
public:
    laser_filter(const std::string& scan_topic, const std::string& filtered_scan_topic)
        : m_nh("laser_filter")
    {
        m_nh.param<float>("min_range", m_min_range, 0.26f);

        m_sub = m_nh.subscribe(scan_topic, 1, &laser_filter::laser_filter_cb, this);
        m_pub = m_nh.advertise<sensor_msgs::LaserScan>(filtered_scan_topic, 1);
    }

    void laser_filter_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        sensor_msgs::LaserScan filtered = *msg;

        for (int i = 0; i < msg->ranges.size(); i++) {
            if (msg->ranges[i] <= m_min_range) {
                filtered.ranges[i] = msg->range_max;
            } else {
                filtered.ranges[i] = msg->ranges[i];
            }
        }

        filtered.range_min = m_min_range;

        m_pub.publish(filtered);
    }

private:
    float m_min_range;

    ros::NodeHandle m_nh;
    ros::Subscriber m_sub;
    ros::Publisher m_pub;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_filter_node");

    laser_filter filter("/scan", "/scan_filtered");

    ROS_INFO("Starting laser filter node");
    ros::Rate rate(100.0f);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

