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

    return 0;
}