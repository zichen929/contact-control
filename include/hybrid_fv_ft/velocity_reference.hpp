#pragma once

#include <chrono>
#include <algorithm>
#include <cmath>

#include <barrett/detail/ca_macro.h>
#include <barrett/systems.h>
#include <barrett/units.h>
#include <Eigen/Core>

namespace hybrid_fv_ft {

// Derivative of a quintic position move: integral over duration equals distance.
inline double moveVelocity(double time, double distance, double duration) {
    if (time <= 0.0 || time >= duration) return 0.0;
    const double s = time/duration;
    return distance/duration * 30.0*s*s*(1.0-s)*(1.0-s);
}

inline double moveDisplacement(double time, double distance, double duration) {
    const double s = std::max(0.0, std::min(1.0, time/duration));
    return distance*s*s*s*(10.0+s*(-15.0+6.0*s));
}

/** Sine velocity or a finite smooth move along one base axis. */
template <typename VelocityVectorType>
class BaseVelocityReference : public barrett::systems::System {
 public:
    typedef barrett::units::CartesianPosition::type position_type;
    Output<VelocityVectorType> output;
    Output<position_type> positionOutput;

    explicit BaseVelocityReference(barrett::systems::ExecutionManager* em,
                                    int baseAxis,
                                    double peakSpeed,
                                    double periodSec,
                                    bool move = false, double distance = 0.10, double moveDuration = 6.0,
                                    const std::string& sysName = "BaseVelocityReference")
        : System(sysName)
        , output(this, &outputValue)
        , positionOutput(this, &positionValue_)
        , baseAxis_(baseAxis)
        , peakSpeed_(peakSpeed)
        , periodSec_(periodSec > 0.0 ? periodSec : 1.0)
        , move_(move), distance_(distance), moveDuration_(moveDuration)
        , haveT0_(false) {
        if (em != NULL) {
            em->startManaging(*this);
        }
        vOut_.setZero();
        origin_.setZero(); position_.setZero();
    }

    virtual ~BaseVelocityReference() {
        this->mandatoryCleanUp();
    }

    void setPeakSpeed(double v) {
        peakSpeed_ = v;
    }

    void setPeriod(double sec) {
        if (sec > 0.0) {
            periodSec_ = sec;
        }
    }

    void setBaseAxis(int axis) {
        baseAxis_ = axis;
    }

    /** Restart phase so motion starts at zero velocity (smooth). */
    void resetPhase(const position_type& origin) {
        origin_ = origin;
        haveT0_ = false;
    }

 protected:
    typename Output<VelocityVectorType>::Value* outputValue;
    VelocityVectorType vOut_;
    Output<position_type>::Value* positionValue_;
    position_type origin_, position_;

    int baseAxis_;
    double peakSpeed_;
    double periodSec_;
    bool move_;
    double distance_, moveDuration_;
    bool haveT0_;

    std::chrono::steady_clock::time_point t0_;

    virtual void operate() {
        using std::chrono::duration;
        using std::chrono::steady_clock;

        if (!haveT0_) {
            t0_ = steady_clock::now();
            haveT0_ = true;
        }

        const double t =
            duration<double>(steady_clock::now() - t0_).count();
        const double w = 2.0 * M_PI / periodSec_;
        const double s = std::sin(w * t);

        vOut_.setZero();
        position_ = origin_;
        if (baseAxis_ >= 0 && baseAxis_ < 3) {
            position_[baseAxis_] += move_ ? moveDisplacement(t, distance_, moveDuration_)
                : peakSpeed_/w*(1.0-std::cos(w*t));
            vOut_[baseAxis_] = move_ ? moveVelocity(t, distance_, moveDuration_) : peakSpeed_ * s;
        }

        outputValue->setData(&vOut_);
        positionValue_->setData(&position_);
    }

 private:
    DISALLOW_COPY_AND_ASSIGN(BaseVelocityReference);
};

}  // namespace hybrid_fv_ft
