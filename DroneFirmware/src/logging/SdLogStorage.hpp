#pragma once

#include "logging/Logger.hpp"

#include <cstddef>
#include <cstdint>

namespace dfw::logging {

class IAppendStorageDevice {
public:
    virtual ~IAppendStorageDevice() = default;

    virtual bool initialize() = 0;
    virtual bool append_bytes(const std::uint8_t* data, std::size_t length) = 0;
};

class SdLogStorage final : public ILogStorage {
public:
    explicit SdLogStorage(IAppendStorageDevice& device);

    bool initialize();
    bool append(const comms::TelemetryFrame& frame) override;
    bool healthy() const;

private:
    struct BinaryLogRecord {
        std::uint32_t magic {0x44464C47U};
        std::uint16_t version {4};
        std::uint16_t reserved {0};
        std::uint64_t timestamp_us {0};
        float roll_rad {0.0f};
        float pitch_rad {0.0f};
        float yaw_rad {0.0f};
        float gyro_rad_s[3] {0.0f, 0.0f, 0.0f};
        float control[4] {0.0f, 0.0f, 0.0f, 0.0f};
        float rate_error_rad_s[3] {0.0f, 0.0f, 0.0f};
        float pid_p[3] {0.0f, 0.0f, 0.0f};
        float pid_i[3] {0.0f, 0.0f, 0.0f};
        float pid_d[3] {0.0f, 0.0f, 0.0f};
        float unsaturated_output[3] {0.0f, 0.0f, 0.0f};
        float saturated_output[3] {0.0f, 0.0f, 0.0f};
        std::uint8_t saturation_flags {0};
        std::uint8_t debug_reserved[3] {0, 0, 0};
        float motors[4] {0.0f, 0.0f, 0.0f, 0.0f};
        std::uint8_t scheduler_mode {0};
        std::uint8_t ai_link_fresh {0};
        std::uint8_t authority_fallback_reason {0};
        std::uint8_t reserved0 {0};
        std::uint32_t authority_transition_count {0};
        std::uint64_t authority_last_transition_us {0};
        std::uint32_t scheduler_skipped_release_count {0};
        std::uint32_t scheduler_slack_denial_count {0};
        std::uint32_t scheduler_mode_transition_count {0};
        std::uint32_t imu_last_us {0};
        std::uint32_t imu_max_us {0};
        std::uint32_t imu_overrun_count {0};
        std::uint32_t imu_skipped_count {0};
        std::uint32_t estimation_last_us {0};
        std::uint32_t estimation_max_us {0};
        std::uint32_t estimation_overrun_count {0};
        std::uint32_t estimation_skipped_count {0};
        std::uint32_t control_last_us {0};
        std::uint32_t control_max_us {0};
        std::uint32_t control_overrun_count {0};
        std::uint32_t control_skipped_count {0};
        std::uint32_t output_last_us {0};
        std::uint32_t output_max_us {0};
        std::uint32_t output_overrun_count {0};
        std::uint32_t output_skipped_count {0};
        std::uint64_t power_sample_time_us {0};
        float power_voltage_v {0.0f};
        float power_current_a {0.0f};
        float power_remaining_ratio {0.0f};
        std::uint8_t power_valid {0};
        std::uint8_t power_status {0};
        std::uint16_t power_reserved {0};
        std::uint64_t imu_health_last_update_us {0};
        std::uint32_t imu_health_total_sample_count {0};
        std::uint32_t imu_health_total_error_count {0};
        std::uint32_t imu_health_consecutive_error_count {0};
        std::uint8_t imu_health_initialized {0};
        std::uint8_t imu_health_healthy {0};
        std::uint8_t imu_health_last_status {0};
        std::uint8_t imu_health_reserved {0};
        std::uint8_t flight_state {0};
        std::uint8_t safety_block_reason {0};
        std::uint8_t safety_allow_motor_output {0};
        std::uint8_t safety_arming_allowed {0};
        std::uint32_t safety_transition_count {0};
        std::uint64_t safety_last_transition_us {0};
        std::uint8_t valid {0};
        std::uint16_t record_size {0};
    };

    static BinaryLogRecord make_record(const comms::TelemetryFrame& frame);

    IAppendStorageDevice& device_;
    bool initialized_ {false};
    bool healthy_ {true};
};

} // namespace dfw::logging
