#pragma once
#include "hybrid_ft/controller_config.hpp"
#include <algorithm>
#include <barrett/systems.h>

namespace hybrid_fp_ft {

using hybrid_ft::ControllerConfig;
// Position PD on selected axes, force feedforward + PI on complementary axes.
class HybridForcePositionControl : public barrett::systems::System {
public:
    typedef barrett::math::Vector<3>::type vector_type;
    typedef barrett::units::CartesianPosition::type position_type;
    Input<position_type> desiredPositionIn, currentPositionIn;
    Input<vector_type> desiredVelocityIn, currentVelocityIn, desiredForceIn, currentForceIn;
    Output<vector_type> controlOutput;
    HybridForcePositionControl(barrett::systems::ExecutionManager* em, const ControllerConfig& config)
        : System("HybridForcePositionControl"), desiredPositionIn(this), currentPositionIn(this),
          desiredVelocityIn(this), currentVelocityIn(this), desiredForceIn(this), currentForceIn(this),
          controlOutput(this, &outputValue_), em_(em), config_(config), enabled_(false) {
        integral_.setZero(); command_.setZero();
        if (em) em->startManaging(*this);
    }
    ~HybridForcePositionControl() { mandatoryCleanUp(); }
    // State changes must be made with the execution-manager mutex held.
    void resetState() { integral_.setZero(); enabled_ = false; }
    void enable() { integral_.setZero(); enabled_ = true; }
    static vector_type calculate(const ControllerConfig& c, const position_type& desired,
            const position_type& current, const vector_type& desiredVelocity,
            const vector_type& velocity, const vector_type& desiredForce,
            const vector_type& force, double dt, vector_type& integral) {
        vector_type result;
        for (int i = 0; i < 3; ++i) {
            if (c.motionAxes[i] == 1) {
                result[i] = c.positionKp[i]*(desired[i]-current[i])
                          + c.positionKd[i]*(desiredVelocity[i]-velocity[i]);
            } else {
                const double error = desiredForce[i]-force[i];
                integral[i] = std::max(-c.integralLimit,
                    std::min(c.integralLimit, integral[i]+error*dt));
                result[i] = c.forceFeedforward*desiredForce[i] + c.kf[i]*error + c.ki[i]*integral[i];
            }
        }
        return result;
    }
protected:
    void operate() override {
        command_.setZero();
        if (enabled_) {
            const double dt = em_ && em_->getPeriod() > 0 ? em_->getPeriod() : 0.001;
            command_ = calculate(config_, desiredPositionIn.getValue(), currentPositionIn.getValue(),
                desiredVelocityIn.getValue(), currentVelocityIn.getValue(), desiredForceIn.getValue(),
                currentForceIn.getValue(), dt, integral_);
        }
        outputValue_->setData(&command_);
    }
private:
    Output<vector_type>::Value* outputValue_;
    barrett::systems::ExecutionManager* em_;
    ControllerConfig config_;
    bool enabled_;
    vector_type integral_, command_;
    DISALLOW_COPY_AND_ASSIGN(HybridForcePositionControl);
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace hybrid_fp_ft
