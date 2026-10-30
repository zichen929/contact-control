#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <barrett/systems.h>

namespace hybrid_fp_ft {
// Unit quintic trajectory: zero velocity and acceleration at both ends.
inline void positionProfile(double time, double duration, double& fraction, double& rate) {
    const double s = std::max(0.0, std::min(1.0, time / duration));
    fraction = s*s*s*(10.0 + s*(-15.0 + 6.0*s));
    rate = 30.0*s*s*(1.0-s)*(1.0-s)/duration;
}

// Blend from the actual start to a sine centered on the captured target.
// The quintic envelope starts with zero velocity, including on a restart.
inline void sinePositionProfile(double time, double rampDuration, double period,
        double amplitude, double start, double origin, double& position, double& velocity) {
    double blend, blendRate;
    positionProfile(time, rampDuration, blend, blendRate);
    const double omega = 2.0 * std::acos(-1.0) / period;
    const double offset = origin - start + amplitude * std::sin(omega*time);
    position = start + blend*offset;
    velocity = blendRate*offset + blend*amplitude*omega*std::cos(omega*time);
}

class BasePositionMove : public barrett::systems::System {
public:
    typedef barrett::units::CartesianPosition::type position_type;
    typedef barrett::math::Vector<3>::type velocity_type;
    Output<position_type> output;
    Output<velocity_type> velocityOutput;
    BasePositionMove(barrett::systems::ExecutionManager* em, double distance, double duration,
                     bool sine = false, double amplitude = 0.05, double period = 6.0)
        : System("BasePositionMove"), output(this, &positionValue_),
          velocityOutput(this, &velocityValue_), distance_(distance), duration_(duration),
          sine_(sine), amplitude_(amplitude), period_(period), active_(false) {
        start_.setZero(); goal_.setZero(); position_.setZero(); velocity_.setZero();
        if (em) em->startManaging(*this);
    }
    ~BasePositionMove() { mandatoryCleanUp(); }
    // Call under the execution-manager mutex. Endpoint is anchored to the t pose,
    // while restart interpolation begins at the actual current position.
    void start(const position_type& current, const position_type& targetOrigin) {
        start_ = current;
        origin_ = targetOrigin;
        goal_ = targetOrigin;
        goal_[0] += distance_;
        t0_ = std::chrono::steady_clock::now();
        active_ = true;
    }
protected:
    void operate() override {
        velocity_.setZero();
        if (active_) {
            double fraction, rate;
            const double time = std::chrono::duration<double>(std::chrono::steady_clock::now()-t0_).count();
            positionProfile(time, duration_, fraction, rate);
            position_ = start_ + fraction*(goal_-start_);
            velocity_ = rate*(goal_-start_);
            if (sine_) {
                position_ = origin_;
                velocity_.setZero();
                sinePositionProfile(time, duration_, period_, amplitude_, start_[0], origin_[0],
                                    position_[0], velocity_[0]);
            }
        }
        positionValue_->setData(&position_);
        velocityValue_->setData(&velocity_);
    }
private:
    Output<position_type>::Value* positionValue_;
    Output<velocity_type>::Value* velocityValue_;
    position_type start_, goal_, origin_, position_;
    velocity_type velocity_;
    double distance_, duration_;
    bool sine_;
    double amplitude_, period_;
    bool active_;
    std::chrono::steady_clock::time_point t0_;
    DISALLOW_COPY_AND_ASSIGN(BasePositionMove);
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace hybrid_fp_ft
