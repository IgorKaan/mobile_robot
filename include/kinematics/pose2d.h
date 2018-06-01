#ifndef MOBILE_ROBOT_COMMS_POSE2D_H
#define MOBILE_ROBOT_COMMS_POSE2D_H

#include <cmath>
#include <ostream>

class pose2d {
public:
    pose2d()
            : m_x(0.0f), m_y(0.0f), m_theta(0.0f) {
    }

    pose2d(float x, float y, float theta)
            : m_x(x), m_y(y), m_theta(theta) {
        normalize_theta();
    }

    ~pose2d() { }

    float get_x() const {
        return m_x;
    }

    void set_x(float x) {
        m_x = x;
    }

    float get_y() const {
        return m_y;
    }

    void set_y(float y) {
        m_y = y;
    }

    float get_theta() const {
        return m_theta;
    }

    void set_theta(float theta) {
        m_theta = theta;
        normalize_theta();
    }
private:
    void normalize_theta() {
        constexpr float pi = M_PI;
        m_theta = m_theta - 2 * pi * std::floor((m_theta + pi) / (2 * pi));
    }

    float m_x = 0.0f;
    float m_y = 0.0f;
    float m_theta = 0.0f;
};


#endif //MOBILE_ROBOT_COMMS_POSE2D_H
