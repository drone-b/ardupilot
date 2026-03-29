#pragma once

#include "control/ControlAllocator.hpp"
#include "control/ControlLoop.hpp"

namespace dfw::control {

class Mixer {
public:
    MotorOutputs mix(const ControlDemand& demand) const;

private:
    static float max_abs(float a, float b, float c, float d);
    static float clamp_unit(float value);
};

} // namespace dfw::control
