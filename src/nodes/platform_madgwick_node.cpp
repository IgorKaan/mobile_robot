#include "madgwick_filter.h"

using namespace platform_imu;

int main(int argc, char **argv) {
    ros::init(argc, argv, "madgwick_node");
    madgwick_filter node;
    ros::spin();
    return 0;
}