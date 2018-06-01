#ifndef MOBILE_ROBOT_COMMS_LINUX_SERIAL_H
#define MOBILE_ROBOT_COMMS_LINUX_SERIAL_H

#include <iostream>
#include <cerrno>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

class linux_serial {
public:
    linux_serial(const char* device_name, speed_t baud_rate);
    linux_serial();
    ~linux_serial();

    void open_port();
    void close_port();

    int write_port(const void* packets, const int count);
    int read_port(void* packets, const int count);

    void set_device_name(const char* device_name);
    char* get_device_name() const;

    void set_port_baud_rate(speed_t baud_rate);
    speed_t get_port_baud_rate() const;
private:
    char* m_port_device_name;

    int m_port_fd {-1};
    speed_t m_port_baud_rate;
};

#endif //MOBILE_ROBOT_COMMS_LINUX_SERIAL_H
