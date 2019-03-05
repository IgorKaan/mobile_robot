#pragma once

#include <cmath>
#include <geometry_msgs/Point.h>

namespace platform_imu {

    class madgwick_filter_library {
    public:
        struct quaternion{
            float x{};
            float y{};
            float z{};
            float w{};
        };
        struct vector{
            float vx{};
            float vy{};
            float vz{};
        };

        madgwick_filter_library();

        void set_algorithm_gain(float gain);
        void set_drift_bias_gain(float zeta);
        void get_orientation(quaternion& quat);
        void set_orientation(quaternion quat);
        void set_quaternion(quaternion quat);


        madgwick_filter_library::quaternion get_quaternion();
        geometry_msgs::Point get_angles_rad_msg();
        geometry_msgs::Point get_angles_deg_msg();

        void madgwick_update_with_magn(float gx, float gy, float gz,
                                       float ax, float ay, float az,
                                       float mx, float my, float mz,
                                       float dt);

        void madgwick_update(float gx, float gy, float gz,
                             float ax, float ay, float az,
                             float dt);

    private:
        float m_gain;    // algorithm gain
        float m_zeta;    // gyro drift bias gain
        //float q0, q1, q2, q3;  // quaternion
        float w_bx_, w_by_, w_bz_;
        quaternion m_quat1;

        void normalize_vector(float *vx,float *vy,float *vz);
        void normalize_quaternion(quaternion *quat);
        void compensate_gyro_drift(quaternion quat, quaternion quat_s,
                                    float dt, float zeta, float& w_bx,
                                    float& w_by, float& w_bz,float& gx,
                                    float& gy, float& gz);
         void rotate_and_scale_vector(quaternion quat, vector _2d, vector *r);
         void add_gradient_descent_step(quaternion quat, vector _2d,
                                        float mx, float my, float mz,
                                        quaternion quat_s);

        madgwick_filter_library::quaternion orientation_change_from_gyro(quaternion quat,
                                                            float gx, float gy, float gz,
                                                            quaternion& qDot);

         void compensate_magnetic_distortion(quaternion quat, float mx, float my, float mz,
                                             float& _2bxy, float& _2bz);


        float get_yaw_rad();
        float get_pitch_rad();
        float get_roll_rad();
        float get_yaw_deg();
        float get_pitch_deg();
        float get_roll_deg();
    };

}
