#pragma once

#include <cmath>
#include <barrett/systems.h>

namespace hybrid_ft {

template <size_t DOF>
class ForceClamp : public barrett::systems::System {
public:
    typedef typename barrett::math::Vector<3>::type base_vector_force_type;

    Input<base_vector_force_type> input;
    Output<base_vector_force_type> output;

    ForceClamp(barrett::systems::ExecutionManager* em,
               const base_vector_force_type& limit,
               const std::string& sysName = "ForceClamp")
        : System(sysName), input(this), output(this, &outputValue), limit_(limit) {
        if (em) em->startManaging(*this);
    }

    virtual ~ForceClamp() { this->mandatoryCleanUp(); }

protected:
    typename Output<base_vector_force_type>::Value* outputValue;
    base_vector_force_type limit_;
    base_vector_force_type f;

    virtual void operate() {
        f = input.getValue();

        for (size_t i = 0; i < 3; ++i) {
            // NaN protection
            if (!std::isfinite(f[i])) {
                f[i] = 0.0;
            }

            // clamp per axis
            if (f[i] > limit_[i])  f[i] = limit_[i];
            if (f[i] < -limit_[i]) f[i] = -limit_[i];
        }

        outputValue->setData(&f);
    }
};



}  // namespace hybrid_ft
