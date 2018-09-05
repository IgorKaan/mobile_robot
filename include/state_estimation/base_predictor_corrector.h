#ifndef MOBILE_ROBOT_COMMS_BASE_PREDICTOR_CORRECTOR_H
#define MOBILE_ROBOT_COMMS_BASE_PREDICTOR_CORRECTOR_H

#include <vector>
#include <Eigen/Dense>
#include <Eigen/StdVector>

template<size_t state_size, size_t control_size, size_t measurement_size>
class base_predictor_corrector {
public:
    template<class eigen_fixed_type>
    using aligned_vector = std::vector<eigen_fixed_type, Eigen::aligned_allocator<eigen_fixed_type>>;

    using state_vector = Eigen::Matrix<float, state_size, 1>;
    using control_vector = Eigen::Matrix<float, control_size, 1>;
    using measurement_vector = Eigen::Matrix<float, measurement_size, 1>;

    using state_container = aligned_vector<state_vector>;
    using control_container = aligned_vector<control_vector>;
    using measurement_container = aligned_vector<measurement_vector>;
public:
    virtual void predict(const control_container& controls) = 0;
    virtual void correct(const measurement_container& measurements) = 0;
private:
    const int m_state_size = state_size;
    const int m_control_size = control_size;
    const int m_measurement_size = measurement_size;
};

#endif //MOBILE_ROBOT_COMMS_BASE_PREDICTOR_CORRECTOR_H
