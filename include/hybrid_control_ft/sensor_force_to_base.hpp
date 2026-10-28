#pragma once

#include <barrett/systems.h>

namespace hybrid_control_ft {

// Sensor axes equal tool axes. World equals base for the identity
// world_to_base configuration. Preserve environment-on-robot force sign.
template <size_t DOF>
class SensorForceToBase : public barrett::systems::SingleIO<
        typename barrett::units::CartesianForce::type,
        typename barrett::units::CartesianForce::type>,
        public barrett::systems::KinematicsInput<DOF> {
    typedef typename barrett::units::CartesianForce::type force_type;

public:
    SensorForceToBase()
        : barrett::systems::SingleIO<force_type, force_type>("SensorForceToBase")
        , barrett::systems::KinematicsInput<DOF>(this) {}

    virtual ~SensorForceToBase() { this->mandatoryCleanUp(); }

protected:
    virtual void operate() {
        rotation_.copyFrom(this->kinInput.getValue().impl->tool->rot_to_world);
        forceBase_ = rotation_ * this->input.getValue();
        this->outputValue->setData(&forceBase_);
    }

private:
    barrett::math::Matrix<3, 3> rotation_;
    force_type forceBase_;
    DISALLOW_COPY_AND_ASSIGN(SensorForceToBase);

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace hybrid_control_ft
