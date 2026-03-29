#include "control/Mixer.hpp"

namespace dfw::control {

MotorOutputs Mixer::mix(const ControlDemand& demand) const
{
    // Quad-X layout:
    // m0: front-right
    // m1: rear-right
    // m2: rear-left
    // m3: front-left
    const float thrust = clamp_unit(demand.thrust);
    const float roll = demand.roll;
    const float pitch = demand.pitch;
    const float yaw = demand.yaw;

    float control_mix[4] {};
    control_mix[0] = -roll - pitch + yaw;
    control_mix[1] = -roll + pitch - yaw;
    control_mix[2] = +roll + pitch + yaw;
    control_mix[3] = +roll - pitch - yaw;

    const float control_peak = max_abs(
        control_mix[0],
        control_mix[1],
        control_mix[2],
        control_mix[3]);

    float scale = 1.0f;
    if (control_peak > 0.0f) {
        const float available_up = 1.0f - thrust;
        const float available_down = thrust;
        const float available_span =
            available_up < available_down ? available_up : available_down;

        if (available_span < control_peak) {
            scale = available_span / control_peak;
        }
    }

    MotorOutputs outputs {};
    for (int i = 0; i < 4; ++i) {
        outputs.values[i] = clamp_unit(thrust + control_mix[i] * scale);
    }

    return outputs;
}

float Mixer::max_abs(float a, float b, float c, float d)
{
    const float values[4] {a, b, c, d};
    float result = 0.0f;

    for (float value : values) {
        const float magnitude = value < 0.0f ? -value : value;
        if (magnitude > result) {
            result = magnitude;
        }
    }

    return result;
}

float Mixer::clamp_unit(float value)
{
    if (value < 0.0f) {
        return 0.0f;
    }

    if (value > 1.0f) {
        return 1.0f;
    }

    return value;
}

} // namespace dfw::control
