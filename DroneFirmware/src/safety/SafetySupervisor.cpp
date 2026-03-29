#include "safety/SafetySupervisor.hpp"

namespace dfw::safety {

SafetySupervisor::SafetySupervisor() = default;

void SafetySupervisor::reset()
{
    state_ = FlightState::disarmed;
    clear_fault_debounce();
}

SafetyOutput SafetySupervisor::update(const SafetyInput& input)
{
    if (input.disarm_command) {
        state_ = FlightState::disarmed;
        clear_fault_debounce();
        SafetyOutput output {};
        output.state = state_;
        output.allow_motor_output = false;
        output.arming_allowed = false;
        output.block_reason = SafetyBlockReason::disarmed;
        return output;
    }

    switch (state_) {
    case FlightState::disarmed:
        clear_fault_debounce();
        if (input.arm_command && arming_conditions_met(input)) {
            state_ = FlightState::armed;
        }
        break;

    case FlightState::armed:
        if (armed_health_valid(input)) {
            clear_fault_debounce();
        } else if (!fault_pending_) {
            fault_pending_ = true;
            fault_started_us_ = input.now_us;
        } else if (input.now_us < fault_started_us_) {
            fault_started_us_ = input.now_us;
        } else if ((input.now_us - fault_started_us_) >= k_fault_debounce_us) {
            state_ = FlightState::failsafe;
            clear_fault_debounce();
        }
        break;

    case FlightState::failsafe:
        // Latched until an explicit disarm command arrives.
        break;
    }

    SafetyOutput output {};
    output.state = state_;
    output.allow_motor_output = state_ == FlightState::armed;
    output.arming_allowed = state_ == FlightState::disarmed && arming_conditions_met(input);

    switch (state_) {
    case FlightState::armed:
        output.block_reason = SafetyBlockReason::none;
        break;

    case FlightState::failsafe:
        output.block_reason = SafetyBlockReason::failsafe;
        break;

    case FlightState::disarmed: {
        const SafetyBlockReason arming_reason = arming_block_reason(input);
        output.block_reason =
            arming_reason == SafetyBlockReason::none ? SafetyBlockReason::disarmed : arming_reason;
        break;
    }
    }

    return output;
}

FlightState SafetySupervisor::state() const
{
    return state_;
}

bool SafetySupervisor::arming_conditions_met(const SafetyInput& input)
{
    return input.command_valid &&
           input.control_output_valid &&
           input.sensor_valid &&
           input.attitude_valid &&
           input.watchdog_healthy &&
           input.throttle_low &&
           input.flight_mode_valid;
}

bool SafetySupervisor::armed_health_valid(const SafetyInput& input)
{
    return input.command_valid &&
           input.control_output_valid &&
           input.sensor_valid &&
           input.attitude_valid &&
           input.watchdog_healthy &&
           input.flight_mode_valid;
}

SafetyBlockReason SafetySupervisor::arming_block_reason(const SafetyInput& input)
{
    if (!input.command_valid) {
        return SafetyBlockReason::invalid_command;
    }

    if (!input.flight_mode_valid) {
        return SafetyBlockReason::invalid_mode;
    }

    if (!input.watchdog_healthy) {
        return SafetyBlockReason::watchdog_fault;
    }

    if (!input.sensor_valid) {
        return SafetyBlockReason::bad_sensor;
    }

    if (!input.attitude_valid) {
        return SafetyBlockReason::bad_attitude;
    }

    if (!input.control_output_valid) {
        return SafetyBlockReason::bad_control;
    }

    if (!input.throttle_low) {
        return SafetyBlockReason::throttle_not_low;
    }

    return SafetyBlockReason::none;
}

void SafetySupervisor::clear_fault_debounce()
{
    fault_pending_ = false;
    fault_started_us_ = 0;
}

} // namespace dfw::safety
