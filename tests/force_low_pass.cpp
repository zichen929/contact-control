#include "hybrid_ft/force_low_pass.hpp"
#include <cassert>
#include <limits>
int main() {
    hybrid_ft::ForceLowPassState state;
    hybrid_ft::ForceLowPassState::vector_type sample;
    sample.setConstant(3);
    assert(state.update(sample,20,.002)[0] == 3); // No artificial startup ramp.
    sample.setConstant(8);
    double first = state.update(sample,20,.002)[0];
    assert(first > 3 && first < 8);
    for (int i=0;i<99;++i) state.update(sample,20,.002);
    const double expected = 8-5*std::exp(-2*std::acos(-1.0)*20*.2);
    assert(std::abs(state.update(sample,20,0)[0]-expected)<1e-12);
    sample.setConstant(-2);
    assert(state.update(sample,0,.002)[0] == -2); // Bypass.
    state.reset();
    sample.setConstant(6);
    assert(state.update(sample,20,.002)[0] == 6);
    sample[0]=std::numeric_limits<double>::quiet_NaN();
    assert(std::isfinite(state.update(sample,20,.002)[0]));
}
