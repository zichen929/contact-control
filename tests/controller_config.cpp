#include "hybrid_control_ft/controller_config.hpp"
#include <iostream>
int main(int argc, char** argv) {
    if (argc != 2) return 2;
    try {
        const auto config = hybrid_control_ft::ControllerConfig::load(argv[1]);
        std::cout << config.kf[2] << " " << config.ki[2] << " "
                  << config.desiredForce[2] << " " << config.peakVelocity << "\n";
        return 0;
    } catch (...) { return 1; }
}
