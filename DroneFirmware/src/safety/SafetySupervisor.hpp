#pragma once

#include "common/TimeTypes.hpp"

namespace dfw::safety {

enum class FlightState : unsigned char {
    disarmed = 0,
    armed,
    failsafe
};

enum class SafetyBlockReason : unsigned char {
    none = 0,
    disarmed,
    failsafe,
    invalid_command,
    bad_sensor,
    bad_attitude,
    bad_control,
    watchdog_fault,
    throttle_not_low,
    invalid_mode
};

struct SafetyInput {
    bool command_valid {false};
    bool arm_command {false};
    bool disarm_command {false};
    bool control_output_valid {false};
    bool sensor_valid {false};
    bool attitude_valid {false};
    bool watchdog_healthy {true};
    bool throttle_low {true};
    bool flight_mode_valid {false};
    common::TimestampUs now_us {0};
};

struct SafetyOutput {
    FlightState state {FlightState::disarmed};
    bool allow_motor_output {false};
    bool arming_allowed {false};
    SafetyBlockReason block_reason {SafetyBlockReason::disarmed};
};

class SafetySupervisor {
public:
    SafetySupervisor();

    void reset();
    SafetyOutput update(const SafetyInput& input);
    FlightState state() const;

private:
    static constexpr common::DurationUs k_fault_debounce_us = 100000;

    static bool arming_conditions_met(const SafetyInput& input);
    static bool armed_health_valid(const SafetyInput& input);
    static SafetyBlockReason arming_block_reason(const SafetyInput& input);

    void clear_fault_debounce();

    FlightState state_ {FlightState::disarmed};
    bool fault_pending_ {false};
    common::TimestampUs fault_started_us_ {0};
};

} // namespace dfw::safety
