#pragma once

#include <barrett/detail/ca_macro.h>
#include <barrett/systems.h>

namespace hybrid_control_ft {

template <size_t DOF>
class BaseControlToJointTorque : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    typedef typename barrett::math::Vector<3>::type base_control_type;
    typedef barrett::math::Matrix<6, DOF> jacobian_type;

    Input<base_control_type> controlBaseIn;
    Input<jacobian_type> jacobianIn;
    Input<jp_type> jointPositionIn;
    Input<jv_type> jointVelocityIn;

    Output<jt_type> jointTorqueOut;

    explicit BaseControlToJointTorque(barrett::systems::ExecutionManager* em,
                                      const v_type& holdKp,
                                      const v_type& holdKd,
                                      const std::string& sysName = "BaseControlToJointTorque")
        : System(sysName)
        , controlBaseIn(this)
        , jacobianIn(this)
        , jointPositionIn(this)
        , jointVelocityIn(this)
        , jointTorqueOut(this, &jointTorqueOutputValue)
        , holdKp_(holdKp), holdKd_(holdKd), holdReady_(false) {
        holdPosition_.setZero();
        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~BaseControlToJointTorque() {
        this->mandatoryCleanUp();
    }

    // Called with the execution-manager mutex held when contact is enabled.
    void setHoldPosition(const jp_type& position) {
        holdPosition_ = position;
        holdReady_ = true;
    }

    // Joints are numbered from 1 on the robot, from 0 in these vectors.
    static jt_type calculateTorque(const jacobian_type& jacobian,
                                   const base_control_type& command,
                                   const jp_type& position,
                                   const jv_type& velocity,
                                   const jp_type& holdPosition,
                                   const v_type& kp, const v_type& kd) {
        jt_type result;
        for (size_t i = 0; i < DOF; ++i) {
            if (i == 1 || i == 3) {
                result[i] = jacobian(0, i) * command[0]
                          + jacobian(1, i) * command[1]
                          + jacobian(2, i) * command[2];
            } else {
                result[i] = kp[i] * (holdPosition[i] - position[i])
                          - kd[i] * velocity[i];
            }
        }
        return result;
    }

  protected:
    typename Output<jt_type>::Value* jointTorqueOutputValue;

    base_control_type controlBase;
    jacobian_type J;
    jt_type tau;

    virtual void operate() {
        // controlBase = controlBaseIn.getValue();
        if (controlBaseIn.valueDefined()) {
            controlBase = controlBaseIn.getValue();
        } else {
            controlBase << 0.0, 0.0, 0.0;
        }
        J = jacobianIn.getValue();

        if (holdReady_) {
            tau = calculateTorque(J, controlBase, jointPositionIn.getValue(),
                                  jointVelocityIn.getValue(), holdPosition_,
                                  holdKp_, holdKd_);
        } else {
            tau.setZero();
        }

        jointTorqueOutputValue->setData(&tau);
    }

  private:
    jp_type holdPosition_;
    v_type holdKp_, holdKd_;
    bool holdReady_;
    DISALLOW_COPY_AND_ASSIGN(BaseControlToJointTorque);
};



}  // namespace hybrid_control_ft
