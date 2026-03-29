#include "control/ControlAllocator.hpp"

namespace dfw::control {

MotorOutputs ControlAllocator::allocate(const ControlDemand& demand)
{
    // Quad-X baseline effectiveness mapping.
    const float thrust_requested = demand.thrust;
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
    std::uint32_t saturated_mask = 0;
    for (int i = 0; i < 4; ++i) {
        const float requested_output = thrust + control_mix[i] * scale;
        const float clamped_output = clamp_unit(requested_output);
        outputs.values[i] = clamped_output;
        if (clamped_output != requested_output) {
            saturated_mask |= (1U << i);
        }
    }

    // Reconstruct achieved normalized wrench from scaled mix for observability.
    const float achieved_roll = roll * scale;
    const float achieved_pitch = pitch * scale;
    const float achieved_yaw = yaw * scale;

    last_status_.unallocated_tau[0] = roll - achieved_roll;
    last_status_.unallocated_tau[1] = pitch - achieved_pitch;
    last_status_.unallocated_tau[2] = yaw - achieved_yaw;
    last_status_.unallocated_thrust = thrust_requested - thrust;
    last_status_.saturated_mask = saturated_mask;
    last_status_.failed_mask = 0;
    last_status_.condition_warning = 0;
    last_status_.solve_iterations = 1;

    return outputs;
}

const AllocatorStatus& ControlAllocator::last_status() const
{
    return last_status_;
}

float ControlAllocator::max_abs(float a, float b, float c, float d)
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

float ControlAllocator::clamp_unit(float value)
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
