#include <cassert>
#include "linux_serial.h"

linux_serial::linux_serial(const char* device_name, speed_t baud_rate)
{
    set_device_name(device_name);
    set_port_baud_rate(baud_rate);
}

linux_serial::linux_serial()
{
    set_device_name("");
    set_port_baud_rate(B9600);
}


linux_serial::~linux_serial() {
    close_port();
}

void linux_serial::open_port() {
    m_port_fd = open(m_port_device_name, O_RDWR | O_NOCTTY | O_NDELAY);

    if (m_port_fd == -1) {
        throw std::runtime_error{"Can't open port"};
    }

    if (!isatty(m_port_fd)) {
        m_port_fd = -1;
        throw std::runtime_error{"Device is not a TTY"};
    }


    termios term_conf;
    if (tcgetattr(m_port_fd, &term_conf) < 0) {
        m_port_fd = -1;
        throw std::runtime_error{"Can't get port config"};
    }

    //term_conf.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
    term_conf.c_oflag &= ~OPOST;
    term_conf.c_lflag &= ~(ECHO | ECHONL | ECHOE | ICANON | IEXTEN | ISIG);
    //term_conf.c_cflag &= ~(CSIZE | CSTOPB | PARENB);
    term_conf.c_cflag |= CLOCAL | CREAD | CS8;

    term_conf.c_cc[VMIN] = 1;
    term_conf.c_cc[VTIME] = 0;

    if (cfsetispeed(&term_conf, m_port_baud_rate) < 0 || cfsetospeed(&term_conf, m_port_baud_rate) < 0) {
        m_port_fd = -1;
        throw std::runtime_error{"Can't set port I/O speed"};
    }

    if (tcsetattr(m_port_fd, TCSAFLUSH, &term_conf) < 0) {
        m_port_fd = -1;
        throw std::runtime_error{"Can't apply config to port"};
    }
}

void linux_serial::close_port() {
    if (m_port_fd != -1) {
        close(m_port_fd);
        m_port_fd = -1;
    }
}

int linux_serial::write_port(const void* packets, const int count) {
    assert(m_port_fd != -1);
    return write(m_port_fd, packets, count);
}

int linux_serial::read_port(void* packets, const int count) {
    assert(m_port_fd != -1);
    return read(m_port_fd, packets, count);
}

void linux_serial::set_device_name(const char* device_name) {
    m_port_device_name = const_cast<char*>(device_name);
}

char *linux_serial::get_device_name() const {
    return m_port_device_name;
}

void linux_serial::set_port_baud_rate(speed_t baud_rate) {
    m_port_baud_rate = baud_rate;
}

speed_t linux_serial::get_port_baud_rate() const {
    return m_port_baud_rate;
}

