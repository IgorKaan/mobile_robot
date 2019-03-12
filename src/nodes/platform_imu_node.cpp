#include "imu.h"

using namespace platform_imu;

const std::string bus = "/dev/i2c-5";

int main(int argc, char **argv) {
    ros::init(argc, argv, "platform_imu_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    imu im(bus);
    ros::Publisher pub = n.advertise<sensor_msgs::Imu>("imu_raw", 1);
    im.init();
    while (ros::ok()){
        im.read_data();
        im.convert_data();
        auto pos = im.get_pos();
        pos.header.frame_id = "base_link";
        pos.header.stamp = ros::Time::now();
        pos.orientation_covariance.elems[0] = -1;

        pos.angular_velocity_covariance.elems[0] = 0.1;
        pos.angular_velocity_covariance.elems[4] = 0.1;
        pos.angular_velocity_covariance.elems[8] = 0.1;

        pos.linear_acceleration_covariance.elems[0] = 0.1;
        pos.linear_acceleration_covariance.elems[4] = 0.1;
        pos.linear_acceleration_covariance.elems[8] = 0.1;

        pub.publish(pos);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
