#include "estimation/AttitudeEstimator.hpp"

#include <cmath>

namespace dfw::estimation {

namespace {

constexpr float k_min_dt_s = 0.00025f;
constexpr float k_max_dt_s = 0.004f;
constexpr float k_accel_correction_gain = 0.02f;

} // namespace

AttitudeEstimator::AttitudeEstimator() = default;

void AttitudeEstimator::reset()
{
    state_ = {};
    previous_sample_time_us_ = 0;
}

bool AttitudeEstimator::update(const sensing::ImuSample& imu_sample)
{
    if (!imu_sample.valid || imu_sample.status != sensing::ImuSampleStatus::ok) {
        state_.valid = false;
        state_.dt_s = 0.0f;
        return false;
    }

    float dt_s = 0.0f;
    if (previous_sample_time_us_ != 0 && imu_sample.sample_time_us > previous_sample_time_us_) {
        const common::DurationUs delta_us =
            static_cast<common::DurationUs>(imu_sample.sample_time_us - previous_sample_time_us_);
        dt_s = static_cast<float>(delta_us) * 1.0e-6f;
    } else if (imu_sample.sample_period_us > 0) {
        dt_s = static_cast<float>(imu_sample.sample_period_us) * 1.0e-6f;
    }

    dt_s = clamp_dt(dt_s);
    previous_sample_time_us_ = imu_sample.sample_time_us;

    state_.roll_rad += imu_sample.gyro_rad_s[0] * dt_s;
    state_.pitch_rad += imu_sample.gyro_rad_s[1] * dt_s;
    state_.yaw_rad += imu_sample.gyro_rad_s[2] * dt_s;

    const float ax = imu_sample.accel_mps2[0];
    const float ay = imu_sample.accel_mps2[1];
    const float az = imu_sample.accel_mps2[2];

    const float accel_roll_rad = std::atan2(ay, az);
    const float accel_pitch_rad = std::atan2(-ax, std::sqrt(ay * ay + az * az));

    state_.roll_rad =
        (1.0f - k_accel_correction_gain) * state_.roll_rad +
        k_accel_correction_gain * accel_roll_rad;
    state_.pitch_rad =
        (1.0f - k_accel_correction_gain) * state_.pitch_rad +
        k_accel_correction_gain * accel_pitch_rad;
    state_.dt_s = dt_s;
    state_.valid = true;
    return true;
}

const AttitudeState& AttitudeEstimator::state() const
{
    return state_;
}

float AttitudeEstimator::clamp_dt(float dt_s)
{
    if (dt_s < k_min_dt_s) {
        return k_min_dt_s;
    }

    if (dt_s > k_max_dt_s) {
        return k_max_dt_s;
    }

    return dt_s;
}

} // namespace dfw::estimation
