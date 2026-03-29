#pragma once

#include "sensing/Sensor.hpp"

namespace dfw::estimation {

struct AttitudeState {
    float roll_rad {0.0f};
    float pitch_rad {0.0f};
    float yaw_rad {0.0f};
    float dt_s {0.0f};
    bool valid {false};
};

class AttitudeEstimator {
public:
    AttitudeEstimator();

    void reset();
    bool update(const sensing::ImuSample& imu_sample);
    const AttitudeState& state() const;

private:
    static float clamp_dt(float dt_s);

    AttitudeState state_ {};
    common::TimestampUs previous_sample_time_us_ {0};
};

} // namespace dfw::estimation
