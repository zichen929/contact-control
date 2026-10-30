#pragma once
#include <cmath>
#include <barrett/systems.h>

namespace hybrid_ft {
// First-order low-pass, exact exponential coefficient for the execution period.
class ForceLowPassState {
public:
    typedef barrett::math::Vector<3>::type vector_type;
    ForceLowPassState() : initialized_(false) { value_.setZero(); }
    void reset() { initialized_ = false; }
    const vector_type& update(const vector_type& sample, double cutoffHz, double dt) {
        const double alpha = cutoffHz > 0 ? -std::expm1(-2.0*std::acos(-1.0)*cutoffHz*dt) : 1.0;
        for (int i = 0; i < 3; ++i) {
            const double input = std::isfinite(sample[i]) ? sample[i] : 0.0;
            value_[i] = !initialized_ || cutoffHz == 0 ? input : value_[i]+alpha*(input-value_[i]);
        }
        initialized_ = true;
        return value_;
    }
private:
    bool initialized_;
    vector_type value_;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class ForceLowPass : public barrett::systems::SingleIO<ForceLowPassState::vector_type,
                                                     ForceLowPassState::vector_type> {
public:
    ForceLowPass(barrett::systems::ExecutionManager* em, double cutoffHz)
        : barrett::systems::SingleIO<ForceLowPassState::vector_type, ForceLowPassState::vector_type>("ForceLowPass"),
          em_(em), cutoffHz_(cutoffHz) {
        if (em_) em_->startManaging(*this);
    }
    ~ForceLowPass() { mandatoryCleanUp(); }
    // Call with execution-manager mutex held after taring the sensor.
    void reset() { state_.reset(); }
protected:
    void operate() override {
        const double dt = em_ && em_->getPeriod() > 0 ? em_->getPeriod() : 0.001;
        this->outputValue->setData(&state_.update(this->input.getValue(), cutoffHz_, dt));
    }
private:
    barrett::systems::ExecutionManager* em_;
    double cutoffHz_;
    ForceLowPassState state_;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace hybrid_ft
