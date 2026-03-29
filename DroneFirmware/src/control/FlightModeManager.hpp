#pragma once

#include "control/InputMapper.hpp"
#include "control/AttitudeController.hpp"

namespace dfw::control {

class FlightModeManager {
public:
    FlightModeManager();

    RateTargetSetpoint update(const PilotCommand& command,
                              const estimation::AttitudeState& attitude_state) const;

private:
    static float clamp(float value, float min_value, float max_value);

    InputMapper input_mapper_ {};
    AttitudeController attitude_controller_ {};
    float max_acro_roll_rate_rad_s_ {4.0f};
    float max_acro_pitch_rate_rad_s_ {4.0f};
    float max_acro_yaw_rate_rad_s_ {2.5f};
};

} // namespace dfw::control
