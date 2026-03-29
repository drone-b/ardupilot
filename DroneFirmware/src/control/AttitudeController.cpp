#include "control/AttitudeController.hpp"

namespace dfw::control {

AttitudeController::AttitudeController() = default;

void AttitudeController::reset()
{
}

RateTargetSetpoint AttitudeController::update(const AttitudeTarget& target,
                                              const estimation::AttitudeState& attitude_state) const
{
    RateTargetSetpoint rate_target {};
    if (!attitude_state.valid) {
        return rate_target;
    }

    const float roll_error_rad = target.roll_target_rad - attitude_state.roll_rad;
    const float pitch_error_rad = target.pitch_target_rad - attitude_state.pitch_rad;

    rate_target.roll_rate_target_rad_s = clamp(
        roll_error_rad * roll_kp_,
        -max_roll_rate_rad_s_,
        max_roll_rate_rad_s_);
    rate_target.pitch_rate_target_rad_s = clamp(
        pitch_error_rad * pitch_kp_,
        -max_pitch_rate_rad_s_,
        max_pitch_rate_rad_s_);
    rate_target.yaw_rate_target_rad_s = target.yaw_rate_target_rad_s;
    rate_target.thrust_target = target.thrust_target;
    rate_target.valid = true;
    return rate_target;
}

float AttitudeController::clamp(float value, float min_value, float max_value)
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
