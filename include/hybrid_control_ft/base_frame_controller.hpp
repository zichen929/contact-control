#pragma once

#include <algorithm>
#include "controller_config.hpp"

#include <barrett/detail/ca_macro.h>
#include <barrett/systems.h>

namespace hybrid_control_ft {

template <size_t DOF>
class HybridForceVelocityControl : public barrett::systems::System {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

 public:
    typedef typename barrett::math::Vector<3>::type base_vector_force_type;
    typedef typename barrett::math::Vector<3>::type base_vector_velocity_type;
    typedef typename barrett::math::Matrix<3, 3>::type selection_matrix_type;
    typedef typename barrett::math::Matrix<3, 3>::type velocity_gain_type;
    typedef typename barrett::math::Matrix<3, 3>::type force_gain_type;
    typedef typename barrett::math::Vector<3>::type base_control_velocity_type;
    typedef typename barrett::math::Vector<3>::type base_control_force_type;
    typedef typename barrett::math::Vector<3>::type base_control_type;

    Input<base_vector_velocity_type> desiredVelocityIn;
    Input<base_vector_velocity_type> currentVelocityIn;

    Input<base_vector_force_type> desiredForceIn;
    Input<base_vector_force_type> currentForceIn;

    Output<base_control_type> controlOutput;

    explicit HybridForceVelocityControl(
        barrett::systems::ExecutionManager* em,
        const ControllerConfig& config,
        const std::string& sysName = "HybridForceVelocityControl")
        : System(sysName)
        , desiredVelocityIn(this)
        , currentVelocityIn(this)
        , desiredForceIn(this)
        , currentForceIn(this)
        , controlOutput(this, &controlOutputValue)
        , em_(em)
        , haveVerrPrev_(false)
        , fIntegralLimit_(config.integralLimit), forceFeedforward_(config.forceFeedforward) {

        if (em != NULL) {
            em->startManaging(*this);
        }

        // Identity
        I.setIdentity();

        S.setZero(); Kv.setZero(); Kd.setZero(); Kf.setZero(); Ki.setZero();
        for (int i = 0; i < 3; ++i) {
            S(i, i) = config.motionAxes[i];
            Kv(i, i) = config.kv[i]; Kd(i, i) = config.kd[i];
            Kf(i, i) = config.kf[i]; Ki(i, i) = config.ki[i];
        }

        fIntegral.setZero();
        vErrPrev.setZero();
    }

    virtual ~HybridForceVelocityControl() {
        this->mandatoryCleanUp();
    }

    void setSelectionMatrix(const selection_matrix_type& selection) {
        S = selection;
    }

    void setVelocityGain(const velocity_gain_type& velocityGain) {
        Kv = velocityGain;
    }

    void setVelocityDerivativeGain(const velocity_gain_type& derivativeGain) {
        Kd = derivativeGain;
    }

    void setForceGain(const force_gain_type& forceGain) {
        Kf = forceGain;
    }

    void setForceIntegralGain(const force_gain_type& integralGain) {
        Ki = integralGain;
    }

    /** Per-axis clamp on integral of force error (|fErr|·s); reduces windup. */
    void setForceIntegralLimit(double limit) {
        fIntegralLimit_ = std::max(0.0, limit);
    }

    /** Clear integral and derivative state (e.g. when enabling/disabling contact). */
    void resetState() {
        fIntegral.setZero();
        vErrPrev.setZero();
        haveVerrPrev_ = false;
    }

 protected:
    typename Output<base_control_type>::Value* controlOutputValue;

    barrett::systems::ExecutionManager* em_;

    base_vector_velocity_type vDes;
    base_vector_velocity_type vCur;
    base_vector_force_type fDes;
    base_vector_force_type fCur;

    base_vector_velocity_type vErr;
    base_vector_force_type fErr;

    base_vector_velocity_type vErrPrev;
    bool haveVerrPrev_;

    base_vector_force_type fIntegral;
    double fIntegralLimit_;
    double forceFeedforward_;

    base_control_velocity_type velocityTerm;
    base_control_force_type forceTerm;
    base_control_type u;

    selection_matrix_type S;
    selection_matrix_type Sf;
    velocity_gain_type Kv;
    velocity_gain_type Kd;
    force_gain_type Kf;
    force_gain_type Ki;
    selection_matrix_type I;

    virtual void operate() {
        const double dt =
            (em_ != NULL && em_->getPeriod() > 0.0) ? em_->getPeriod() : 0.001;

        vDes = desiredVelocityIn.getValue();
        vCur = currentVelocityIn.getValue();

        fDes = desiredForceIn.getValue();
        // fCur = currentForceIn.getValue();
        if (currentForceIn.valueDefined()) {
            fCur = currentForceIn.getValue();
        } else {
            fCur.setZero();
        }

        Sf = I - S;

        vErr = vDes - vCur;
        fErr = fDes - fCur;

        base_vector_velocity_type vErrDot;
        if (!haveVerrPrev_) {
            vErrPrev = vErr;
            haveVerrPrev_ = true;
            vErrDot.setZero();
        } else {
            vErrDot = (vErr - vErrPrev) / dt;
            vErrPrev = vErr;
        }

        base_vector_force_type fErrSel = Sf * fErr;
        fIntegral += fErrSel * dt;
        if (fIntegralLimit_ > 0.0) {
            for (int i = 0; i < 3; ++i) {
                fIntegral[i] =
                    std::max(-fIntegralLimit_,
                             std::min(fIntegralLimit_, fIntegral[i]));
            }
        }

        velocityTerm = S * (Kv * vErr + Kd * vErrDot);
        // Select force axes after combining feedforward and feedback.
        forceTerm = Sf * (forceFeedforward_ * fDes + Kf * fErr + Ki * fIntegral);

        u = velocityTerm + forceTerm;

        controlOutputValue->setData(&u);
    }

 private:
    DISALLOW_COPY_AND_ASSIGN(HybridForceVelocityControl);
};


}  // namespace hybrid_control_ft
