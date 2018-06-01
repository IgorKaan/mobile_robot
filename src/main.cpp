#include <iostream>
#include <cerrno>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#include "linux_serial.h"
#include "kinematics/differential_drive.h"

/*
 * socat -d -d pty,raw,echo=0 pty,raw,echo=0
 */

int main() {
    /*
    try {
        linux_serial serial("/dev/pts/23", B9600);
        serial.open_port();

        std::cout << "Opened " << serial.get_device_name() << std::endl;

        serial.write_port("ATZ\r", 4);

        unsigned char status = 'K';
        while (true) {
            if (serial.read_port(&status, 1) > 0) {
                write(STDOUT_FILENO, &status, 1);
                if (status == 'Q') {
                    break;
                }
            }
        }

        serial.close_port();
    } catch (std::runtime_error& e) {
        std::cout << "Runtime error: " << e.what() << std::endl;
    }
    */

    // TODO: do more forward kinematics checks

    pose2d pose(0.0, 0.0, 0.0);

    differential_drive::parameters parms;
    parms.wheel_radius = 0.05f;
    parms.axis_length = 0.15f;

    /*
     * Rotate about left wheel
     * left_rpm = 0
     * right_rpm > 0
     *
     * Rotate about right wheel
     * left_rpm > 0
     * right_rpm = 0
     *
     * Forward linear motion
     * left_rpm == right_rpm
     *
     * Rotate in place
     * left_rpm == -right_rpm
     */
    float left_rpm = 150;
    float right_rpm = 150;

    float left_omega = (M_2_PI / 60) * left_rpm;
    float right_omega = (M_2_PI / 60) * right_rpm;

    float dt = 0.333f;
    differential_drive::pose_with_twist pose_twist = differential_drive::forward_kinematics(pose, parms,
                                                                                          left_omega, right_omega, dt);

    pose2d new_pose = pose_twist.pose;
    differential_drive::twist2d twist = pose_twist.twist;

    std::cout << "pose:  " << new_pose.get_x() << ", " << new_pose.get_y() << ", " << new_pose.get_theta() << std::endl;
    std::cout << "twist: " << twist.vx << ", " << twist.vy << ", " << twist.omega << std::endl;

    return 0;
}