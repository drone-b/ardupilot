#include "control/InputMapper.hpp"

namespace dfw::control {

InputMapper::InputMapper() = default;

AttitudeTarget InputMapper::map(const PilotInput& input) const
{
    AttitudeTarget target {};
    if (!input.valid) {
        return target;
    }

    const float roll_input = clamp(input.roll, -1.0f, 1.0f);
    const float pitch_input = clamp(input.pitch, -1.0f, 1.0f);
    const float yaw_input = clamp(input.yaw, -1.0f, 1.0f);
    const float throttle_input = clamp(input.throttle, 0.0f, 1.0f);

    const config::ParameterRegistry& parameters = config::ParameterRegistry::instance();
    float max_roll_angle_rad = 0.523599f;
    float max_pitch_angle_rad = 0.523599f;
    float max_yaw_rate_rad_s = 2.5f;
    parameters.get_float(config::ParameterId::input_max_roll_angle_rad, max_roll_angle_rad);
    parameters.get_float(config::ParameterId::input_max_pitch_angle_rad, max_pitch_angle_rad);
    parameters.get_float(config::ParameterId::input_max_yaw_rate_rad_s, max_yaw_rate_rad_s);

    target.roll_target_rad = roll_input * max_roll_angle_rad;
    target.pitch_target_rad = pitch_input * max_pitch_angle_rad;
    target.yaw_rate_target_rad_s = yaw_input * max_yaw_rate_rad_s;
    target.thrust_target = throttle_input;
    return target;
}

float InputMapper::clamp(float value, float min_value, float max_value)
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
