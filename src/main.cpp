#include <iostream>
#include <cerrno>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <state_estimation/base_predictor_corrector.h>

#include "kinematics/differential_drive.h"

int main() {
    base_predictor_corrector<6, 3, 3>::state_vector vec;

    for (int i = 0; i < vec.rows(); i++) {
        vec(i) = i * 3.14f;
    }

    base_predictor_corrector<6, 3, 3>::state_container states;

    states.push_back(vec);
    states.push_back(vec);

    for (const auto& state : states) {
        std::cout << state << std::endl;
    }

    return 0;
}