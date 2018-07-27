#include <iostream>
#include <cerrno>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#include "kinematics/differential_drive.h"

int main() {

    M_2_PI;

    // Record rosbag
    // rosbag record -O mylaserdata /base_scan /tf
    // before playing
    // rosparam set use_sim_time true
    // rosbag play --clock <name of the bag>
    // Or just save the map after mapping
    // rosrun map_server map_saver -f <map_name>


    return 0;
}