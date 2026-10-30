#include "hybrid_ft/joint_torque.hpp"
#include <cassert>
#include <cmath>

// Hardware-free regression: Cartesian commands cannot leak into held joints,
// and position-hold errors cannot alter the two planar joint commands.
template <size_t DOF>
void checkPlanarTorque() {
    typedef hybrid_ft::BaseControlToJointTorque<DOF> Controller;
    typename Controller::jacobian_type J;
    J.setOnes();
    typename Controller::base_control_type force;
    force << 2.0, 0.0, -5.0;
    typename barrett::units::JointPositions<DOF>::type q, hold;
    typename barrett::units::JointVelocities<DOF>::type velocity;
    typename barrett::math::Vector<DOF>::type kp, kd;
    q.setZero(); hold.setConstant(0.1); velocity.setConstant(0.2);
    kp.setConstant(10.0); kd.setConstant(2.0);
    const auto torque = Controller::calculateTorque(J, force, q, velocity, hold, kp, kd);
    force *= 10.0;
    const auto stronger = Controller::calculateTorque(J, force, q, velocity, hold, kp, kd);
    for (size_t i = 0; i < DOF; ++i) {
        const bool active = i == 1 || i == 3;
        assert(std::abs(torque[i] - (active ? -3.0 : 0.6)) < 1e-10);
        assert(std::abs(stronger[i] - (active ? -30.0 : 0.6)) < 1e-10);
    }
    hold = q; velocity.setZero();
    const auto settled = Controller::calculateTorque(J, force, q, velocity, hold, kp, kd);
    for (size_t i = 0; i < DOF; ++i) {
        assert(std::abs(settled[i] - ((i == 1 || i == 3) ? -30.0 : 0.0)) < 1e-10);
    }
}
int main() {
    checkPlanarTorque<4>();
    checkPlanarTorque<7>();
}
