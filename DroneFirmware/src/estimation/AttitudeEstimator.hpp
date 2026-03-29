#pragma once

#include "sensing/Sensor.hpp"

#include <cstdint>

namespace dfw::estimation {

struct AttitudeState {
    float q[4] {1.0f, 0.0f, 0.0f, 0.0f};
    float roll_rad {0.0f};
    float pitch_rad {0.0f};
    float yaw_rad {0.0f};
    float gyro_bias_rad_s[3] {0.0f, 0.0f, 0.0f};
    float innovation_norm {0.0f};
    float innovation_norm_xyz[3] {0.0f, 0.0f, 0.0f};
    float covariance_diag[6] {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    std::uint8_t gate_status {0};
    std::uint8_t estimator_lane {0};
    std::uint8_t estimator_health {0};
    bool innovation_gated {false};
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
    struct Quaternion {
        float w {1.0f};
        float x {0.0f};
        float y {0.0f};
        float z {0.0f};
    };

    static float clamp_dt(float dt_s);
    static float clamp(float value, float min_value, float max_value);
    static float sqrtf_safe(float value);
    static void normalize_quaternion(Quaternion& q);
    static Quaternion integrate_body_rate(const Quaternion& q,
                                          const float omega_rad_s[3],
                                          float dt_s);
    static void quaternion_to_euler(const Quaternion& q,
                                    float& roll_rad,
                                    float& pitch_rad,
                                    float& yaw_rad);

    AttitudeState state_ {};
    Quaternion q_ {};
    float covariance_diag_[6] {0.02f, 0.02f, 0.04f, 0.001f, 0.001f, 0.001f};
    std::uint32_t consecutive_gated_cycles_ {0};
    std::uint32_t consecutive_clean_cycles_ {0};
    common::TimestampUs previous_sample_time_us_ {0};
};

} // namespace dfw::estimation
