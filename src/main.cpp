#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>

/*
 * socat -d -d pty,raw,echo=0 pty,raw,echo=0
 */
const char* device = "/dev/pts/21";

int main() {
    termios term_conf;

    int serial_descriptor = open(device, O_RDWR | O_NOCTTY | O_NDELAY);

    if (serial_descriptor == -1) {
        std::cout << "rip port" << std::endl;
        return -1;
    }

    if (!isatty(serial_descriptor)) {
        std::cout << "not a tty" << std::endl;
        return -1;
    }

    if (tcgetattr(serial_descriptor, &term_conf) < 0) {
        std::cout << "can't get serial interface configuration" << std::endl;
        return -1;
    }

    term_conf.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
    term_conf.c_oflag = 0;
    term_conf.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
    term_conf.c_cflag &= ~(CSIZE | PARENB);
    term_conf.c_cflag |= CS8;

    term_conf.c_cc[VMIN] = 1;
    term_conf.c_cc[VTIME] = 1;

    if (cfsetispeed(&term_conf, B9600) < 0 || cfsetospeed(&term_conf, B9600) < 0) {
        std::cout << "Can't set serial input/output speed" << std::endl;
        return -1;
    }

    if (tcsetattr(serial_descriptor, TCSAFLUSH, &term_conf) < 0) {
        std::cout << "Can't apply current config for current serial port" << std::endl;
        return -1;
    }

    std::cout << "Success!" << std::endl;

    unsigned char status = 'K';

    while (status != 'Q') {
        if (read(serial_descriptor, &status, 1) > 0) {
            write(STDOUT_FILENO, &status, 1);
            std::cout << status << std::endl;
        }

        if (read(STDIN_FILENO, &status, 1) > 0) {
            write(serial_descriptor, &status, 1);
        }
    }

    close(serial_descriptor);

    return 0;
}