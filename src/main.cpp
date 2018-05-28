#include <iostream>
#include <cerrno>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

/*
 * socat -d -d pty,raw,echo=0 pty,raw,echo=0
 */
const char* device = "/dev/pts/2";

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

    //term_conf.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
    term_conf.c_oflag &= ~OPOST;
    term_conf.c_lflag &= ~(ECHO | ECHONL | ECHOE | ICANON | IEXTEN | ISIG);
    //term_conf.c_cflag &= ~(CSIZE | CSTOPB | PARENB);
    term_conf.c_cflag |= CLOCAL | CREAD | CS8;

    term_conf.c_cc[VMIN] = 1;
    term_conf.c_cc[VTIME] = 0;

    if (cfsetispeed(&term_conf, B38400) < 0 || cfsetospeed(&term_conf, B38400) < 0) {
        std::cout << "Can't set serial input/output speed" << std::endl;
        return -1;
    }

    if (tcsetattr(serial_descriptor, TCSAFLUSH, &term_conf) < 0) {
        std::cout << "Can't apply current config for current serial port" << std::endl;
        return -1;
    }

    //fcntl(serial_descriptor, F_SETFL, 0);

    std::cout << "Success!" << std::endl;


    write(serial_descriptor, "ATZ\r", 4);

    unsigned char status = 'K';
    while (true) {
        if (read(serial_descriptor, &status, 1) > 0) {
            write(STDOUT_FILENO, &status, 1);
            if (status == 'Q') {
                break;
            }
        }
    }

    close(serial_descriptor);

    return 0;
}