#include "imu.h"

using namespace platform_imu;

const std::string bus = "/dev/i2c-4";

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    imu im(bus);
    ros::Publisher pub = n.advertise<sensor_msgs::Imu>("imu", 1);
    im.init();
    while (ros::ok()){
        im.read_data();
        im.convert_data();
        pub.publish(im.get_pos());
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
