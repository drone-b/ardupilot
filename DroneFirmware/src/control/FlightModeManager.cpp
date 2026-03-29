#include "control/FlightModeManager.hpp"

namespace dfw::control {

FlightModeManager::FlightModeManager() = default;

RateTargetSetpoint FlightModeManager::update(const PilotCommand& command,
                                             const estimation::AttitudeState& attitude_state) const
{
    RateTargetSetpoint rate_target {};
    if (!command.valid || !command.input.valid || command.disarm) {
        return rate_target;
    }

    switch (command.mode) {
    case FlightMode::stabilize: {
        const AttitudeTarget attitude_target = input_mapper_.map(command.input);
        return attitude_controller_.update(attitude_target, attitude_state);
    }

    case FlightMode::acro:
        rate_target.roll_rate_target_rad_s = clamp(
            command.input.roll,
            -1.0f,
            1.0f) * max_acro_roll_rate_rad_s_;
        rate_target.pitch_rate_target_rad_s = clamp(
            command.input.pitch,
            -1.0f,
            1.0f) * max_acro_pitch_rate_rad_s_;
        rate_target.yaw_rate_target_rad_s = clamp(
            command.input.yaw,
            -1.0f,
            1.0f) * max_acro_yaw_rate_rad_s_;
        rate_target.thrust_target = clamp(command.input.throttle, 0.0f, 1.0f);
        rate_target.valid = true;
        return rate_target;
    }

    return rate_target;
}

float FlightModeManager::clamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

} // namespace dfw::control
