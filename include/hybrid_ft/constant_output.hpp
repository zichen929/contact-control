#pragma once
#include <barrett/systems.h>

namespace hybrid_ft {

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


} // namespace hybrid_ft
