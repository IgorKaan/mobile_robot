#include <madgwick_filter_library.h>
#include <cmath>

using namespace platform_imu;

madgwick_filter_library::madgwick_filter_library() :
        w_bx_(0.0), w_by_(0.0), w_bz_(0.0),
        m_zeta (0.0), m_gain (0.1) {}

void madgwick_filter_library::set_algorithm_gain(float gain) {
    m_gain = gain;
}

void madgwick_filter_library::set_drift_bias_gain(float zeta) {
    m_zeta = zeta;
}

void madgwick_filter_library::get_orientation(quaternion& quat) {
    float q0, q1, q2, q3;
    quat.w = this->m_quat1.w;
    quat.x = this->m_quat1.x;
    quat.y = this->m_quat1.y;
    quat.z = this->m_quat1.z;

    float recip_norm = 1 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    m_quat1.w *= recip_norm;
    m_quat1.x *= recip_norm;
    m_quat1.y *= recip_norm;
    m_quat1.z *= recip_norm;
}

void madgwick_filter_library::set_orientation(quaternion quat) {
    this->m_quat1.w = quat.w;
    this->m_quat1.x = quat.x;
    this->m_quat1.y = quat.y;
    this->m_quat1.z = quat.z;

//    w_bx_ = 0;
//    w_by_ = 0;
//    w_bz_ = 0;
}

void madgwick_filter_library::set_quaternion(quaternion quat) {
    float q1 = quat.x;
    float q2 = quat.y;
    float q3 = quat.z;
    float q0 = quat.w;
}

madgwick_filter_library::quaternion madgwick_filter_library::get_quaternion() {
    float q0, q1, q2, q3;
    m_quat1.x = q1;
    m_quat1.y = q2;
    m_quat1.z = q3;
    m_quat1.w = q0;

    return  m_quat1;
}


void madgwick_filter_library::normalize_vector(float *vx,float *vy,float *vz) {
    float recip_norm = sqrt(*vx * *vx + *vy * *vy + *vz * *vz);
    *vx *= recip_norm;
    *vy *= recip_norm;
    *vz *= recip_norm;
}

void madgwick_filter_library::normalize_quaternion(quaternion *quat) {
    float recip_norm = sqrt(quat->w * quat->w + quat->x * quat->x +
                            quat->y * quat->y + quat->z * quat->z);
    quat->x *= recip_norm;
    quat->y *= recip_norm;
    quat->z *= recip_norm;
    quat->w *= recip_norm;
}

void  madgwick_filter_library::rotate_and_scale_vector(quaternion quat, vector _2d, vector *r) {

    // result is half as long as input
    r->vx = _2d.vx * (0.5f - quat.y * quat.y - quat.z * quat.z)
         + _2d.vy * (quat.w * quat.z + quat.x * quat.y)
         + _2d.vz * (quat.x * quat.z - quat.w * quat.y);
    r->vy = _2d.vx * (quat.x * quat.y - quat.w * quat.z)
         + _2d.vy * (0.5f - quat.x * quat.x - quat.z * quat.z)
         + _2d.vz * (quat.w * quat.x + quat.y * quat.z);
    r->vz = _2d.vx * (quat.w * quat.y + quat.x * quat.z)
         + _2d.vy * (quat.y * quat.z - quat.w * quat.x)
         + _2d.vz * (0.5f - quat.x * quat.x - quat.y * quat.y);
}


void  madgwick_filter_library::compensate_gyro_drift(quaternion quat, quaternion quat_s,
                                                    float dt, float zeta, float& w_bx,
                                                    float& w_by, float& w_bz,
                                                    float& gx, float& gy,
                                                    float& gz) {
    // w_err = 2 q x s
    float w_err_x = 2.0f * quat.w * quat_s.x - 2.0f * quat.x * quat_s.w -
                    2.0f * quat.y * quat_s.z + 2.0f * quat.z * quat_s.y;
    float w_err_y = 2.0f * quat.w * quat_s.y + 2.0f * quat.x * quat_s.z -
                    2.0f * quat.y * quat_s.w - 2.0f * quat.z * quat_s.x;
    float w_err_z = 2.0f * quat.w * quat_s.z - 2.0f * quat.x * quat_s.y +
                    2.0f * quat.y * quat_s.x - 2.0f * quat.z * quat_s.w;

    w_bx += w_err_x * dt * zeta;
    w_by += w_err_y * dt * zeta;
    w_bz += w_err_z * dt * zeta;

    gx -= w_bx;
    gy -= w_by;
    gz -= w_bz;
}

void madgwick_filter_library::add_gradient_descent_step(quaternion quat, vector _2d,
                                                        float mx, float my, float mz,
                                                        quaternion quat_s) {

    madgwick_filter_library::vector vec_f;

    // Gradient decent algorithm corrective step
    rotate_and_scale_vector(quat, _2d, &vec_f);

    vec_f.vx -= mx;
    vec_f.vy -= my;
    vec_f.vz -= mz;

    // Jt * f
    quat_s.w += (_2d.vy * quat.z - _2d.vz * quat.y) * vec_f.vx
                + (-_2d.vx * quat.z + _2d.vz * quat.x) * vec_f.vy
                + (_2d.vx * quat.y - _2d.vy * quat.x) * vec_f.vz;

    quat_s.x += (_2d.vy * quat.y + _2d.vz * quat.z) * vec_f.vx
                + (_2d.vx * quat.y - 2.0f * _2d.vy * quat.x + _2d.vz * quat.w) * vec_f.vy
                + (_2d.vx * quat.z - _2d.vy * quat.w - 2.0f * _2d.vz * quat.x) * vec_f.vz;

    quat_s.y += (-2.0f * _2d.vx * quat.y + _2d.vy * quat.x - _2d.vz * quat.w) * vec_f.vx
                + (_2d.vx * quat.x + _2d.vz * quat.z) * vec_f.vy
                + (_2d.vx * quat.w + _2d.vy * quat.y - 2.0f * _2d.vz * quat.y) * vec_f.vz;

    quat_s.z += (-2.0f * _2d.vx * quat.z + _2d.vy * quat.w + _2d.vz * quat.x) * vec_f.vx
                + (-_2d.vx * quat.w - 2.0f * _2d.vy * quat.z + _2d.vz * quat.y) * vec_f.vy
                + (_2d.vx * quat.x + _2d.vy * quat.y) * vec_f.vz;
}

madgwick_filter_library::quaternion madgwick_filter_library::orientation_change_from_gyro(quaternion quat,
                                                                                float gx, float gy, float gz,
                                                                                quaternion& qDot) {
    // Rate of change of quaternion from gyroscope
    qDot.w = 0.5f * (-quat.x * gx - quat.y * gy - quat.z * gz);
    qDot.x = 0.5f * (quat.w * gx + quat.y * gz - quat.z * gy);
    qDot.y = 0.5f * (quat.w * gy - quat.x * gz + quat.z * gx);
    qDot.z = 0.5f * (quat.w * gz + quat.x * gy -quat.y * gx);

    return qDot;
}



void madgwick_filter_library::compensate_magnetic_distortion(quaternion quat,
                                                            float mx, float my, float mz,
                                                            float& _2bxy, float& _2bz) {
    vector h;
    vector vec_m;
    mx = vec_m.vx;
    my = vec_m.vy;
    mz = vec_m.vz;
    // Reference direction of Earth's magnetic field
    rotate_and_scale_vector(quat, vec_m, &h);
    quat.x = -quat.x;
    quat.y = -quat.y;
    quat.z = -quat.z;
    quat.w = quat.w;

    _2bxy = 4.0f * sqrt (h.vx * h.vx + h.vy * h.vy);
    _2bz = 4.0f * h.vz;

}

// algoritm with magnetometr
void madgwick_filter_library::madgwick_update_with_magn(
        float gx, float gy, float gz,
        float ax, float ay, float az,
        float mx, float my, float mz,
        float dt) {
    float _2bz, _2bxy;
    quaternion quat_s, q_Dot;
    vector _2b, _2d;
    _2b.vx = _2bxy; _2b.vy = 0.0f; _2b.vz = _2bz;
    _2d.vx = 0.0f; _2d.vy = 0.0f; _2d.vz = 2.0f;


    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if (!std::isfinite(mx) || !std::isfinite(my) || !std::isfinite(mz))
    {
        madgwick_update(gx, gy, gz, ax, ay, az, dt);
        return;
    }

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accelerometer measurement
        normalize_vector(&ax, &ay, &az);
        // Normalise magnetometer measurement
        normalize_vector(&mx, &my, &mz);
        // Compensate for magnetic distortion
        compensate_magnetic_distortion(m_quat1, mx, my, mz, _2bxy, _2bz);
        // Gradient decent algorithm corrective step
        quat_s.x = 0.0;  quat_s.y = 0.0;  quat_s.z = 0.0;  quat_s.w = 0.0;
        // Gravity: [0, 0, 1]
        add_gradient_descent_step(m_quat1, _2d, ax, ay, az, quat_s);
        // Earth magnetic field: = [bxy, 0, bz]
        add_gradient_descent_step(m_quat1, _2b, mx, my, mz, quat_s);
        normalize_quaternion(&quat_s);
        // compute gyro drift bias
        compensate_gyro_drift(m_quat1, quat_s, dt, m_zeta, w_bx_, w_by_, w_bz_, gx, gy, gz);
        orientation_change_from_gyro(m_quat1, gx, gy, gz, q_Dot);
        // Apply feedback step
        q_Dot.w -= m_gain * quat_s.w;
        q_Dot.x -= m_gain * quat_s.x;
        q_Dot.y -= m_gain * quat_s.y;
        q_Dot.z -= m_gain * quat_s.z;
    }
    else
    {
        orientation_change_from_gyro(m_quat1, gx, gy, gz, q_Dot);
    }
    // Integrate rate of change of quaternion to yield quaternion
    m_quat1.w += q_Dot.w * dt;
    m_quat1.x += q_Dot.x * dt;
    m_quat1.y += q_Dot.y * dt;
    m_quat1.z += q_Dot.z * dt;
    // Normalise quaternion
    normalize_quaternion(&quat_s);
}

// algoritm without magnetometr
void madgwick_filter_library::madgwick_update(float gx, float gy, float gz, float ax,
                                              float ay,float az, float dt) {
    float recip_norm;
    quaternion quat_s, q_Dot;
    vector _2d;
    _2d.vx = 0.0f; _2d.vy = 0.0f; _2d.vz = 2.0f;
    // Rate of change of quaternion from gyroscope
    orientation_change_from_gyro (m_quat1, gx, gy, gz, q_Dot);
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accelerometer measurement
        normalize_vector(&ax, &ay, &az);
        // Gradient decent algorithm corrective step
        quat_s.x = 0.0;  quat_s.y = 0.0;  quat_s.z = 0.0;  quat_s.w = 0.0;
        // Gravity: [0, 0, 1]
        add_gradient_descent_step(m_quat1, _2d, ax, ay, az, quat_s);
        normalize_quaternion(&quat_s);
        // Apply feedback step
        q_Dot.w -= m_gain * quat_s.w;
        q_Dot.x -= m_gain * quat_s.x;
        q_Dot.y -= m_gain * quat_s.y;
        q_Dot.z -= m_gain * quat_s.z;
    }

    // Integrate rate of change of quaternion to yield quaternion
    m_quat1.w += q_Dot.w * dt;
    m_quat1.x += q_Dot.x * dt;
    m_quat1.y += q_Dot.y * dt;
    m_quat1.z += q_Dot.z * dt;
    // Normalise quaternion
    normalize_quaternion (&m_quat1);
}

float madgwick_filter_library::get_yaw_rad(){
    return atan2(2 * m_quat1.x * m_quat1.y - 2 * m_quat1.w * m_quat1.z,
            2 * m_quat1.w * m_quat1.w + 2 * m_quat1.x * m_quat1.x - 1);
}

float madgwick_filter_library::get_pitch_rad(){
    return atan2(2 * m_quat1.y * m_quat1.z - 2 * m_quat1.w * m_quat1.x,
            2 * m_quat1.w * m_quat1.w + 2 * m_quat1.z * m_quat1.z - 1);
}

float madgwick_filter_library::get_roll_rad(){
    return -1 * atan2(2.0f * (m_quat1.w * m_quat1.y - m_quat1.x * m_quat1.z),
            1.0f - 2.0f * (m_quat1.y * m_quat1.y + m_quat1.x *m_quat1.x ));
}

// CLion don't like type float. So using 'static_cast<float>'
float madgwick_filter_library::get_yaw_deg(){
    return static_cast<float>(atan2(2 * m_quat1.x * m_quat1.y - 2 * m_quat1.w * m_quat1.z,
                                    2 * m_quat1.w * m_quat1.w + 2 * m_quat1.x * m_quat1.x - 1) * 180.0f / M_PI);
}

float madgwick_filter_library::get_pitch_deg(){
    return static_cast<float>(atan2(2 * m_quat1.y * m_quat1.z - 2 * m_quat1.w * m_quat1.x,
                                    2 * m_quat1.w * m_quat1.w + 2 * m_quat1.z * m_quat1.z - 1) * 180.0f / M_PI);
}

float madgwick_filter_library::get_roll_deg(){
    return static_cast<float>(-1 * atan2(2.0f * (m_quat1.w * m_quat1.y - m_quat1.x * m_quat1.z),
                                         1.0f - 2.0f * (m_quat1.y * m_quat1.y + m_quat1.x *m_quat1.x )) * 180.0f / M_PI);
}


geometry_msgs::Point madgwick_filter_library::get_angles_rad_msg(){
    geometry_msgs::Point point;
    point.x = get_roll_rad();
    point.y = get_pitch_rad();
    point.z = get_yaw_rad();

    return point;

}

geometry_msgs::Point madgwick_filter_library::get_angles_deg_msg(){
    geometry_msgs::Point point;
    point.x = get_roll_deg();
    point.y = get_pitch_deg();
    point.z = get_yaw_deg();

    return point;

}





