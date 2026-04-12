#pragma once

#include "common/TimeTypes.hpp"
#include "control/ControlAllocator.hpp"
#include "control/ControlLoop.hpp"
#include "estimation/AttitudeEstimator.hpp"
#include "platform/Hal.hpp"
#include "safety/SafetySupervisor.hpp"
#include "sensing/PowerMonitor.hpp"
#include "sensing/Sensor.hpp"

#include <cstdint>

namespace dfw::comms {

struct SchedulerRuntimeSnapshot {
    common::DurationUs imu_last_us {0};
    common::DurationUs imu_max_us {0};
    std::uint32_t imu_overrun_count {0};
    std::uint32_t imu_skipped_count {0};

    common::DurationUs estimation_last_us {0};
    common::DurationUs estimation_max_us {0};
    std::uint32_t estimation_overrun_count {0};
    std::uint32_t estimation_skipped_count {0};

    common::DurationUs control_last_us {0};
    common::DurationUs control_max_us {0};
    std::uint32_t control_overrun_count {0};
    std::uint32_t control_skipped_count {0};

    common::DurationUs output_last_us {0};
    common::DurationUs output_max_us {0};
    std::uint32_t output_overrun_count {0};
    std::uint32_t output_skipped_count {0};

    std::uint32_t scheduler_skipped_release_count {0};
    std::uint32_t scheduler_slack_denial_count {0};
    std::uint32_t scheduler_mode_transition_count {0};
};

struct TelemetryFrame {
    common::TimestampUs timestamp_us {0};
    estimation::AttitudeState attitude {};
    sensing::ImuSample imu_sample {};
    sensing::SensorHealth imu_health {};
    control::ControlDemand control_demand {};
    control::ControlLoopDebug control_debug {};
    control::MotorOutputs motor_outputs {};
    control::AllocatorStatus allocator_status {};
    std::uint8_t scheduler_mode {0};
    std::uint8_t ai_link_fresh {0};
    std::uint8_t authority_fallback_reason {0};
    std::uint32_t authority_transition_count {0};
    common::TimestampUs authority_last_transition_us {0};
    SchedulerRuntimeSnapshot scheduler_runtime {};
    sensing::PowerSample power_sample {};
    safety::FlightState flight_state {safety::FlightState::disarmed};
    safety::SafetyBlockReason safety_block_reason {safety::SafetyBlockReason::disarmed};
    std::uint8_t safety_allow_motor_output {0};
    std::uint8_t safety_arming_allowed {0};
    std::uint32_t safety_transition_count {0};
    common::TimestampUs safety_last_transition_us {0};
};

class TelemetryPublisher {
public:
    struct PublishStats {
        std::uint32_t attempted_frame_count {0};
        std::uint32_t accepted_frame_count {0};
        std::uint32_t busy_frame_count {0};
        std::uint32_t failed_frame_count {0};
        std::uint32_t total_write_bytes {0};
        std::uint16_t last_write_size {0};
        platform::StatusCode last_status {platform::StatusCode::ok};
    };

    explicit TelemetryPublisher(platform::IUartPort& uart_port);

    void publish(const TelemetryFrame& frame);
    const PublishStats& stats() const;

private:
    struct __attribute__((packed)) BinaryTelemetryFrame {
        std::uint8_t magic_le[4] {0, 0, 0, 0};
        std::uint8_t version_le[2] {0, 0};
        std::uint8_t frame_size_le[2] {0, 0};
        std::uint8_t sequence_le[4] {0, 0, 0, 0};
        std::uint8_t timestamp_us_le[8] {0, 0, 0, 0, 0, 0, 0, 0};
        std::uint8_t attitude_rad_le[3][4] {};
        std::uint8_t gyro_rad_s_le[3][4] {};
        std::uint8_t control_le[4][4] {};
        std::uint8_t rate_error_rad_s_le[3][4] {};
        std::uint8_t pid_p_le[3][4] {};
        std::uint8_t pid_i_le[3][4] {};
        std::uint8_t pid_d_le[3][4] {};
        std::uint8_t unsaturated_output_le[3][4] {};
        std::uint8_t saturated_output_le[3][4] {};
        std::uint8_t saturation_flags {0};
        std::uint8_t debug_reserved[3] {0, 0, 0};
        std::uint8_t motors_le[4][4] {};
        std::uint8_t flight_state {0};
        std::uint8_t reserved[3] {0, 0, 0};
        std::uint8_t authority_transition_count_le[4] {0, 0, 0, 0};
        std::uint8_t authority_last_transition_us_le[8] {0, 0, 0, 0, 0, 0, 0, 0};
        std::uint8_t scheduler_skipped_release_count_le[4] {0, 0, 0, 0};
        std::uint8_t scheduler_slack_denial_count_le[4] {0, 0, 0, 0};
        std::uint8_t scheduler_mode_transition_count_le[4] {0, 0, 0, 0};
        std::uint8_t imu_last_us_le[4] {0, 0, 0, 0};
        std::uint8_t imu_max_us_le[4] {0, 0, 0, 0};
        std::uint8_t imu_overrun_count_le[4] {0, 0, 0, 0};
        std::uint8_t imu_skipped_count_le[4] {0, 0, 0, 0};
        std::uint8_t estimation_last_us_le[4] {0, 0, 0, 0};
        std::uint8_t estimation_max_us_le[4] {0, 0, 0, 0};
        std::uint8_t estimation_overrun_count_le[4] {0, 0, 0, 0};
        std::uint8_t estimation_skipped_count_le[4] {0, 0, 0, 0};
        std::uint8_t control_last_us_le[4] {0, 0, 0, 0};
        std::uint8_t control_max_us_le[4] {0, 0, 0, 0};
        std::uint8_t control_overrun_count_le[4] {0, 0, 0, 0};
        std::uint8_t control_skipped_count_le[4] {0, 0, 0, 0};
        std::uint8_t output_last_us_le[4] {0, 0, 0, 0};
        std::uint8_t output_max_us_le[4] {0, 0, 0, 0};
        std::uint8_t output_overrun_count_le[4] {0, 0, 0, 0};
        std::uint8_t output_skipped_count_le[4] {0, 0, 0, 0};
        std::uint8_t power_sample_time_us_le[8] {0, 0, 0, 0, 0, 0, 0, 0};
        std::uint8_t power_voltage_v_le[4] {0, 0, 0, 0};
        std::uint8_t power_current_a_le[4] {0, 0, 0, 0};
        std::uint8_t power_remaining_ratio_le[4] {0, 0, 0, 0};
        std::uint8_t power_valid {0};
        std::uint8_t power_status {0};
        std::uint8_t power_reserved[2] {0, 0};
        std::uint8_t imu_health_last_update_us_le[8] {0, 0, 0, 0, 0, 0, 0, 0};
        std::uint8_t imu_health_total_sample_count_le[4] {0, 0, 0, 0};
        std::uint8_t imu_health_total_error_count_le[4] {0, 0, 0, 0};
        std::uint8_t imu_health_consecutive_error_count_le[4] {0, 0, 0, 0};
        std::uint8_t imu_health_initialized {0};
        std::uint8_t imu_health_healthy {0};
        std::uint8_t imu_health_last_status {0};
        std::uint8_t imu_health_reserved {0};
        std::uint8_t safety_block_reason {0};
        std::uint8_t safety_allow_motor_output {0};
        std::uint8_t safety_arming_allowed {0};
        std::uint8_t safety_reserved {0};
        std::uint8_t safety_transition_count_le[4] {0, 0, 0, 0};
        std::uint8_t safety_last_transition_us_le[8] {0, 0, 0, 0, 0, 0, 0, 0};
    };

    struct __attribute__((packed)) BinaryTelemetryHeader {
        std::uint8_t sync_word_le[2] {0, 0};
        std::uint8_t payload_size_le[2] {0, 0};
        std::uint8_t crc32_le[4] {0, 0, 0, 0};
    };

    struct __attribute__((packed)) FramedTelemetryPacket {
        BinaryTelemetryHeader header {};
        BinaryTelemetryFrame payload {};
    };

    static BinaryTelemetryFrame make_binary_frame(const TelemetryFrame& frame,
                                                  std::uint32_t sequence);
    static FramedTelemetryPacket make_framed_packet(const TelemetryFrame& frame,
                                                    std::uint32_t sequence);
    static std::uint32_t compute_crc32(const BinaryTelemetryFrame& payload);
    static void write_le16(std::uint8_t* destination, std::uint16_t value);
    static void write_le32(std::uint8_t* destination, std::uint32_t value);
    static void write_le64(std::uint8_t* destination, std::uint64_t value);
    static void write_float_le(std::uint8_t* destination, float value);

    platform::IUartPort& uart_port_;
    std::uint32_t sequence_counter_ {0};
    PublishStats stats_ {};
};

} // namespace dfw::comms
