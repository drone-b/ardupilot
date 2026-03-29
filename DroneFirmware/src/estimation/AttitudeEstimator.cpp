#include "estimation/AttitudeEstimator.hpp"

#include <cmath>

namespace dfw::estimation {

namespace {

constexpr float k_min_dt_s = 0.00025f;
constexpr float k_max_dt_s = 0.004f;
constexpr float k_accel_correction_gain = 0.06f;
constexpr float k_bias_adaptation_gain = 0.02f;
constexpr float k_innovation_gate_rad = 0.8f;
constexpr float k_pi_over_two = 1.5707963267948966f;

} // namespace

AttitudeEstimator::AttitudeEstimator() = default;

void AttitudeEstimator::reset()
{
    state_ = {};
    q_ = {};
    previous_sample_time_us_ = 0;
}

bool AttitudeEstimator::update(const sensing::ImuSample& imu_sample)
{
    if (!imu_sample.valid || imu_sample.status != sensing::ImuSampleStatus::ok) {
        state_.valid = false;
        state_.dt_s = 0.0f;
        state_.estimator_health = 2U;
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

    float omega_body[3] {
        imu_sample.gyro_rad_s[0] - state_.gyro_bias_rad_s[0],
        imu_sample.gyro_rad_s[1] - state_.gyro_bias_rad_s[1],
        imu_sample.gyro_rad_s[2] - state_.gyro_bias_rad_s[2],
    };
    q_ = integrate_body_rate(q_, omega_body, dt_s);

    const float ax = imu_sample.accel_mps2[0];
    const float ay = imu_sample.accel_mps2[1];
    const float az = imu_sample.accel_mps2[2];

    const float accel_norm = sqrtf_safe(ax * ax + ay * ay + az * az);
    if (accel_norm > 1.0e-3f) {
        const float accel_roll_rad = std::atan2(ay, az);
        const float accel_pitch_rad = std::atan2(-ax, sqrtf_safe(ay * ay + az * az));

        float predicted_roll_rad = 0.0f;
        float predicted_pitch_rad = 0.0f;
        float predicted_yaw_rad = 0.0f;
        quaternion_to_euler(q_, predicted_roll_rad, predicted_pitch_rad, predicted_yaw_rad);

        const float innovation_roll = accel_roll_rad - predicted_roll_rad;
        const float innovation_pitch = accel_pitch_rad - predicted_pitch_rad;
        state_.innovation_norm_xyz[0] = innovation_roll < 0.0f ? -innovation_roll : innovation_roll;
        state_.innovation_norm_xyz[1] = innovation_pitch < 0.0f ? -innovation_pitch : innovation_pitch;
        state_.innovation_norm_xyz[2] = 0.0f;
        state_.innovation_norm = sqrtf_safe(
            innovation_roll * innovation_roll + innovation_pitch * innovation_pitch);
        state_.innovation_gated = state_.innovation_norm > k_innovation_gate_rad;
        state_.gate_status = state_.innovation_gated ? 0x00U : 0x01U;

        if (!state_.innovation_gated) {
            // Small-angle correction using the accelerometer gravity reference.
            const float corrected_roll =
                predicted_roll_rad + innovation_roll * k_accel_correction_gain;
            const float corrected_pitch =
                predicted_pitch_rad + innovation_pitch * k_accel_correction_gain;

            state_.gyro_bias_rad_s[0] += innovation_roll * dt_s * k_bias_adaptation_gain;
            state_.gyro_bias_rad_s[1] += innovation_pitch * dt_s * k_bias_adaptation_gain;

            // Rebuild quaternion from corrected roll/pitch and propagated yaw.
            const float cr = std::cos(corrected_roll * 0.5f);
            const float sr = std::sin(corrected_roll * 0.5f);
            const float cp = std::cos(corrected_pitch * 0.5f);
            const float sp = std::sin(corrected_pitch * 0.5f);
            const float cy = std::cos(predicted_yaw_rad * 0.5f);
            const float sy = std::sin(predicted_yaw_rad * 0.5f);

            q_.w = cr * cp * cy + sr * sp * sy;
            q_.x = sr * cp * cy - cr * sp * sy;
            q_.y = cr * sp * cy + sr * cp * sy;
            q_.z = cr * cp * sy - sr * sp * cy;
            normalize_quaternion(q_);
        }
    } else {
        state_.innovation_norm = 0.0f;
        state_.innovation_gated = true;
        state_.innovation_norm_xyz[0] = 0.0f;
        state_.innovation_norm_xyz[1] = 0.0f;
        state_.innovation_norm_xyz[2] = 0.0f;
        state_.gate_status = 0x00U;
    }

    quaternion_to_euler(q_, state_.roll_rad, state_.pitch_rad, state_.yaw_rad);
    state_.q[0] = q_.w;
    state_.q[1] = q_.x;
    state_.q[2] = q_.y;
    state_.q[3] = q_.z;
    state_.pitch_rad = clamp(state_.pitch_rad, -k_pi_over_two, k_pi_over_two);
    state_.dt_s = dt_s;
    state_.estimator_lane = 0U;
    state_.estimator_health = state_.innovation_gated ? 1U : 0U;
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

float AttitudeEstimator::clamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

float AttitudeEstimator::sqrtf_safe(float value)
{
    return value > 0.0f ? std::sqrt(value) : 0.0f;
}

void AttitudeEstimator::normalize_quaternion(Quaternion& q)
{
    const float norm = sqrtf_safe(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm <= 1.0e-6f) {
        q = {};
        return;
    }

    q.w /= norm;
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;
}

AttitudeEstimator::Quaternion AttitudeEstimator::integrate_body_rate(const Quaternion& q,
                                                                     const float omega_rad_s[3],
                                                                     float dt_s)
{
    // First-order quaternion integration from body angular velocity.
    Quaternion out {};
    const float half_dt = 0.5f * dt_s;
    const float wx = omega_rad_s[0];
    const float wy = omega_rad_s[1];
    const float wz = omega_rad_s[2];

    out.w = q.w + (-q.x * wx - q.y * wy - q.z * wz) * half_dt;
    out.x = q.x + (+q.w * wx + q.y * wz - q.z * wy) * half_dt;
    out.y = q.y + (+q.w * wy - q.x * wz + q.z * wx) * half_dt;
    out.z = q.z + (+q.w * wz + q.x * wy - q.y * wx) * half_dt;
    normalize_quaternion(out);
    return out;
}

void AttitudeEstimator::quaternion_to_euler(const Quaternion& q,
                                            float& roll_rad,
                                            float& pitch_rad,
                                            float& yaw_rad)
{
    const float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    const float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    roll_rad = std::atan2(sinr_cosp, cosr_cosp);

    const float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    if (sinp >= 1.0f) {
        pitch_rad = k_pi_over_two;
    } else if (sinp <= -1.0f) {
        pitch_rad = -k_pi_over_two;
    } else {
        pitch_rad = std::asin(sinp);
    }

    const float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    const float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    yaw_rad = std::atan2(siny_cosp, cosy_cosp);
}

} // namespace dfw::estimation
