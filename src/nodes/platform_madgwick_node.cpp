#include "madgwick_filter.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

using namespace platform_imu;

int main(int argc, char **argv) {
    ros::init(argc, argv, "madgwick_node");
    madgwick_filter node;
    ros::NodeHandle n = node.get_node_handle();
    message_filters::Subscriber<nav_msgs::Odometry> odometry_sub(n, "odom", 10);
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub(n, "imu_data", 10);

    message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::Imu> sync(odometry_sub, imu_sub, 20);
    auto cb_bind = boost::bind(&madgwick_filter::odom_imu_sync, boost::ref(node), _1, _2);

    sync.registerCallback(cb_bind);

    ros::spin();
    return 0;
}