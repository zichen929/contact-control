#include "hybrid_fv_ft/velocity_reference.hpp"
#include <cassert>
#include <cmath>
int main() {
    using hybrid_fv_ft::moveVelocity;
    using hybrid_fv_ft::moveDisplacement;
    assert(moveDisplacement(0, .1, 6) == 0);
    assert(std::abs(moveDisplacement(3, .1, 6)-.05) < 1e-12);
    assert(std::abs(moveDisplacement(60, .1, 6)-.1) < 1e-12);
    assert(moveVelocity(-1, .1, 6) == 0);
    assert(moveVelocity(0, .1, 6) == 0);
    assert(moveVelocity(6, .1, 6) == 0);
    assert(moveVelocity(60, .1, 6) == 0);
    assert(std::abs(moveVelocity(3, .1, 6)-0.03125) < 1e-12);
    // Numerically verify the commanded displacement for both directions.
    for (double distance : {0.10, -0.20, 0.0}) {
        double displacement = 0;
        const double dt = 6.0/10000;
        for (int i = 0; i < 10000; ++i)
            displacement += moveVelocity((i+0.5)*dt, distance, 6)*dt;
        assert(std::abs(displacement-distance) < 1e-9);
    }
}
