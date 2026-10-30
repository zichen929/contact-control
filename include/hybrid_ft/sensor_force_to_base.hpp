#pragma once

#include <barrett/systems.h>

namespace hybrid_ft {

// Sensor axes equal tool axes. World equals base for the identity
// world_to_base configuration. Preserve environment-on-robot force sign.
template <size_t DOF>
class SensorForceToBase : public barrett::systems::SingleIO<
        typename barrett::units::CartesianForce::type,
        typename barrett::units::CartesianForce::type>,
        public barrett::systems::KinematicsInput<DOF> {
    typedef typename barrett::units::CartesianForce::type force_type;

public:
    typedef barrett::math::Matrix<3, 3> rotation_type;
    barrett::systems::System::Output<rotation_type> rotationOutput;

    SensorForceToBase()
        : barrett::systems::SingleIO<force_type, force_type>("SensorForceToBase")
        , barrett::systems::KinematicsInput<DOF>(this)
        , rotationOutput(this, &rotationOutputValue_) {}

    virtual ~SensorForceToBase() { this->mandatoryCleanUp(); }

protected:
    virtual void operate() {
        rotation_.copyFrom(this->kinInput.getValue().impl->tool->rot_to_world);
        forceBase_ = rotation_ * this->input.getValue();
        this->outputValue->setData(&forceBase_);
        rotationOutputValue_->setData(&rotation_);
    }

private:
    barrett::systems::System::Output<rotation_type>::Value* rotationOutputValue_;
    barrett::math::Matrix<3, 3> rotation_;
    force_type forceBase_;
    DISALLOW_COPY_AND_ASSIGN(SensorForceToBase);

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace hybrid_ft
