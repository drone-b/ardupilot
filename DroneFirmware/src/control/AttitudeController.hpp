#pragma once

#include "estimation/AttitudeEstimator.hpp"

namespace dfw::control {

struct AttitudeTarget {
    float roll_target_rad {0.0f};
    float pitch_target_rad {0.0f};
    float yaw_rate_target_rad_s {0.0f};
    float thrust_target {0.0f};
};

struct RateTargetSetpoint {
    float roll_rate_target_rad_s {0.0f};
    float pitch_rate_target_rad_s {0.0f};
    float yaw_rate_target_rad_s {0.0f};
    float thrust_target {0.0f};
    bool valid {false};
};

class AttitudeController {
public:
    AttitudeController();

    void reset();
    RateTargetSetpoint update(const AttitudeTarget& target,
                              const estimation::AttitudeState& attitude_state) const;

private:
    static float clamp(float value, float min_value, float max_value);

    float roll_kp_ {4.0f};
    float pitch_kp_ {4.0f};
    float max_roll_rate_rad_s_ {3.0f};
    float max_pitch_rate_rad_s_ {3.0f};
};

} // namespace dfw::control
