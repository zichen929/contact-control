#include "hybrid_fp_ft/controller.hpp"
#include "hybrid_fp_ft/position_reference.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

int main(int argc, char** argv) {
    using namespace hybrid_ft;
    using namespace hybrid_fp_ft;
    assert(argc == 2);
    auto c = ControllerConfig::load(argv[1], true);
    double position, velocity;
    sinePositionProfile(0, 6, 6, 0.05, 0.6, 0.5, position, velocity);
    assert(position == 0.6 && velocity == 0);
    sinePositionProfile(7.5, 6, 6, 0.05, 0.6, 0.5, position, velocity);
    assert(std::abs(position-0.55) < 1e-12 && std::abs(velocity) < 1e-12);
    sinePositionProfile(10.5, 6, 6, 0.05, 0.6, 0.5, position, velocity);
    assert(std::abs(position-0.45) < 1e-12);
    // Analytic feedforward velocity must match the position derivative during ramp.
    double before, after, unused;
    sinePositionProfile(2, 6, 6, 0.05, 0.6, 0.5, position, velocity);
    sinePositionProfile(2-1e-5, 6, 6, 0.05, 0.6, 0.5, before, unused);
    sinePositionProfile(2+1e-5, 6, 6, 0.05, 0.6, 0.5, after, unused);
    assert(std::abs(velocity-(after-before)/2e-5) < 1e-8);
    double fraction, rate;
    positionProfile(0, 6, fraction, rate);
    assert(fraction == 0 && rate == 0);
    positionProfile(3, 6, fraction, rate);
    assert(std::abs(fraction-0.5) < 1e-12 && rate > 0);
    double previous = 0;
    for (int i = 0; i <= 600; ++i) {
        positionProfile(i/100.0, 6, fraction, rate);
        assert(fraction >= previous-1e-12 && fraction <= 1 && rate >= 0);
        previous = fraction;
    }
    positionProfile(60, 6, fraction, rate);
    assert(fraction == 1 && rate == 0); // Endpoint held, no periodic restart.
    typedef HybridForcePositionControl C;
    C::position_type desired, current;
    C::vector_type vd, v, fd, f, integral;
    desired.setZero(); current.setZero(); vd.setZero(); v.setZero();
    fd.setZero(); f.setZero(); integral.setZero();
    c.positionKp = {{100,0,0}}; c.positionKd = {{20,0,0}};
    c.kf = {{0,0,0.25}}; c.ki = {{0,0,0.1}};
    c.forceFeedforward = 1; c.integralLimit = 25;
    desired[0] = 0.1; desired[2] = 100; // z position must not affect force control.
    fd[2] = -5; f[2] = -5;
    auto u = C::calculate(c, desired,current,vd,v,fd,f,0.002,integral);
    assert(std::abs(u[0]-10) < 1e-12 && u[1] == 0 && u[2] == -5);
    v[0] = 0.1;
    u = C::calculate(c, desired,current,vd,v,fd,f,0.002,integral);
    assert(std::abs(u[0]-8) < 1e-12); // Damping opposes motion.
    current[0] = desired[0]; v.setZero();
    u = C::calculate(c, desired,current,vd,v,fd,f,0.002,integral);
    assert(u[0] == 0); // At endpoint, no residual motion command.
    f[2] = 0;
    for (int i = 0; i < 10000; ++i)
        u = C::calculate(c, desired,current,vd,v,fd,f,0.002,integral);
    assert(integral[0] == 0 && integral[2] == -25);
    assert(std::abs(u[2]+8.75) < 1e-12);
    std::cout << "Position profile and hybrid position/force law passed\n";
}
