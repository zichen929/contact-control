#pragma once

#include <barrett/products/force_torque_sensor.h>
#include <barrett/systems.h>

namespace hybrid_control_ft {

// Read environment-on-robot force in sensor coordinates once per cycle.
// No rotation, sign change, automatic tare, or payload compensation. Moments are unused.
template <size_t DOF>
class BarrettFTForce : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
    Output<cf_type> output;

    explicit BarrettFTForce(barrett::ForceTorqueSensor& sensor)
        : System("BarrettFTForce")
        , output(this, &outputValue_)
        , sensor_(sensor) {}

    virtual ~BarrettFTForce() { this->mandatoryCleanUp(); }

protected:
    virtual void operate() {
        sensor_.update(true);
        forceSensor_ = sensor_.getForce();
        outputValue_->setData(&forceSensor_);
    }

private:
    typename Output<cf_type>::Value* outputValue_;
    barrett::ForceTorqueSensor& sensor_;
    cf_type forceSensor_;
    DISALLOW_COPY_AND_ASSIGN(BarrettFTForce);

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace hybrid_control_ft
