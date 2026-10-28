#pragma once

#include <chrono>
#include <cmath>

#include <barrett/detail/ca_macro.h>
#include <barrett/systems.h>
#include <barrett/units.h>
#include <Eigen/Core>

namespace hybrid_control_ft {

template <typename T>
class ConstantOutput : public barrett::systems::System {
 public:
    Output<T> output;

    explicit ConstantOutput(barrett::systems::ExecutionManager* em,
                            const T& initialValue,
                            const std::string& sysName = "ConstantOutput")
        : System(sysName)
        , output(this, &outputValue)
        , value(initialValue) {
        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~ConstantOutput() {
        this->mandatoryCleanUp();
    }

    void setValue(const T& newValue) {
        value = newValue;
    }

 protected:
    typename Output<T>::Value* outputValue;
    T value;

    virtual void operate() {
        outputValue->setData(&value);
    }

 private:
    DISALLOW_COPY_AND_ASSIGN(ConstantOutput);
};


/** Smooth back-and-forth base-frame velocity along one axis: v(t) = peakSpeed * sin(2*pi*t/T). */
template <typename VelocityVectorType>
class BaseVelocitySinusoid : public barrett::systems::System {
 public:
    Output<VelocityVectorType> output;

    explicit BaseVelocitySinusoid(barrett::systems::ExecutionManager* em,
                                    int baseAxis,
                                    double peakSpeed,
                                    double periodSec,
                                    const std::string& sysName = "BaseVelocitySinusoid")
        : System(sysName)
        , output(this, &outputValue)
        , baseAxis_(baseAxis)
        , peakSpeed_(peakSpeed)
        , periodSec_(periodSec > 0.0 ? periodSec : 1.0)
        , haveT0_(false) {
        if (em != NULL) {
            em->startManaging(*this);
        }
        vOut_.setZero();
    }

    virtual ~BaseVelocitySinusoid() {
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
    void resetPhase() {
        haveT0_ = false;
    }

 protected:
    typename Output<VelocityVectorType>::Value* outputValue;
    VelocityVectorType vOut_;

    int baseAxis_;
    double peakSpeed_;
    double periodSec_;
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
        if (baseAxis_ >= 0 && baseAxis_ < 3) {
            vOut_[baseAxis_] = peakSpeed_ * s;
        }

        outputValue->setData(&vOut_);
    }

 private:
    DISALLOW_COPY_AND_ASSIGN(BaseVelocitySinusoid);
};

}  // namespace hybrid_control_ft
