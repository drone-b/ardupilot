#pragma once

#include "control/ControlLoop.hpp"

#include <cstdint>

namespace dfw::control {

struct MotorOutputs {
    float values[4] {0.0f, 0.0f, 0.0f, 0.0f};
};

struct AllocatorStatus {
    float unallocated_tau[3] {0.0f, 0.0f, 0.0f};
    float unallocated_thrust {0.0f};
    std::uint32_t saturated_mask {0};
    std::uint32_t failed_mask {0};
    std::uint32_t condition_warning {0};
    std::uint32_t solve_iterations {0};
};

class ControlAllocator {
public:
    MotorOutputs allocate(const ControlDemand& demand);
    const AllocatorStatus& last_status() const;

private:
    static float max_abs(float a, float b, float c, float d);
    static float clamp_unit(float value);

    AllocatorStatus last_status_ {};
};

} // namespace dfw::control
