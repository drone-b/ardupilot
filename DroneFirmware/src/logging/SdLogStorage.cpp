#include "logging/SdLogStorage.hpp"

namespace dfw::logging {

SdLogStorage::SdLogStorage(IAppendStorageDevice& device) :
    device_(device)
{
}

bool SdLogStorage::initialize()
{
    initialized_ = device_.initialize();
    healthy_ = initialized_;
    return initialized_;
}

bool SdLogStorage::append(const comms::TelemetryFrame& frame)
{
    if (!initialized_ || !healthy_) {
        return false;
    }

    const BinaryLogRecord record = make_record(frame);
    const bool write_ok = device_.append_bytes(
        reinterpret_cast<const std::uint8_t*>(&record),
        sizeof(record));

    if (!write_ok) {
        healthy_ = false;
    }

    return write_ok;
}

bool SdLogStorage::healthy() const
{
    return healthy_;
}

SdLogStorage::BinaryLogRecord SdLogStorage::make_record(const comms::TelemetryFrame& frame)
{
    BinaryLogRecord record {};
    record.timestamp_us = frame.timestamp_us;
    record.roll_rad = frame.attitude.roll_rad;
    record.pitch_rad = frame.attitude.pitch_rad;
    record.yaw_rad = frame.attitude.yaw_rad;
    record.gyro_rad_s[0] = frame.imu_sample.gyro_rad_s[0];
    record.gyro_rad_s[1] = frame.imu_sample.gyro_rad_s[1];
    record.gyro_rad_s[2] = frame.imu_sample.gyro_rad_s[2];
    record.control[0] = frame.control_demand.roll;
    record.control[1] = frame.control_demand.pitch;
    record.control[2] = frame.control_demand.yaw;
    record.control[3] = frame.control_demand.thrust;
    record.rate_error_rad_s[0] = frame.control_debug.roll.rate_error_rad_s;
    record.rate_error_rad_s[1] = frame.control_debug.pitch.rate_error_rad_s;
    record.rate_error_rad_s[2] = frame.control_debug.yaw.rate_error_rad_s;
    record.pid_p[0] = frame.control_debug.roll.proportional;
    record.pid_p[1] = frame.control_debug.pitch.proportional;
    record.pid_p[2] = frame.control_debug.yaw.proportional;
    record.pid_i[0] = frame.control_debug.roll.integrator;
    record.pid_i[1] = frame.control_debug.pitch.integrator;
    record.pid_i[2] = frame.control_debug.yaw.integrator;
    record.pid_d[0] = frame.control_debug.roll.derivative;
    record.pid_d[1] = frame.control_debug.pitch.derivative;
    record.pid_d[2] = frame.control_debug.yaw.derivative;
    record.unsaturated_output[0] = frame.control_debug.roll.unsaturated_output;
    record.unsaturated_output[1] = frame.control_debug.pitch.unsaturated_output;
    record.unsaturated_output[2] = frame.control_debug.yaw.unsaturated_output;
    record.saturated_output[0] = frame.control_debug.roll.saturated_output;
    record.saturated_output[1] = frame.control_debug.pitch.saturated_output;
    record.saturated_output[2] = frame.control_debug.yaw.saturated_output;
    record.saturation_flags =
        (frame.control_debug.roll.saturated ? 0x01U : 0U) |
        (frame.control_debug.pitch.saturated ? 0x02U : 0U) |
        (frame.control_debug.yaw.saturated ? 0x04U : 0U);
    record.debug_reserved[0] =
        static_cast<std::uint8_t>(frame.allocator_status.saturated_mask & 0xFFU);
    record.debug_reserved[1] =
        static_cast<std::uint8_t>(frame.allocator_status.condition_warning & 0xFFU);
    record.debug_reserved[2] =
        static_cast<std::uint8_t>(frame.allocator_status.solve_iterations & 0xFFU);
    record.motors[0] = frame.motor_outputs.values[0];
    record.motors[1] = frame.motor_outputs.values[1];
    record.motors[2] = frame.motor_outputs.values[2];
    record.motors[3] = frame.motor_outputs.values[3];
    record.scheduler_mode = frame.scheduler_mode;
    record.ai_link_fresh = frame.ai_link_fresh;
    record.authority_fallback_reason = frame.authority_fallback_reason;
    record.scheduler_skipped_release_count =
        frame.scheduler_runtime.scheduler_skipped_release_count;
    record.scheduler_slack_denial_count =
        frame.scheduler_runtime.scheduler_slack_denial_count;
    record.scheduler_mode_transition_count =
        frame.scheduler_runtime.scheduler_mode_transition_count;
    record.imu_last_us = frame.scheduler_runtime.imu_last_us;
    record.imu_max_us = frame.scheduler_runtime.imu_max_us;
    record.imu_overrun_count = frame.scheduler_runtime.imu_overrun_count;
    record.imu_skipped_count = frame.scheduler_runtime.imu_skipped_count;
    record.estimation_last_us = frame.scheduler_runtime.estimation_last_us;
    record.estimation_max_us = frame.scheduler_runtime.estimation_max_us;
    record.estimation_overrun_count = frame.scheduler_runtime.estimation_overrun_count;
    record.estimation_skipped_count = frame.scheduler_runtime.estimation_skipped_count;
    record.control_last_us = frame.scheduler_runtime.control_last_us;
    record.control_max_us = frame.scheduler_runtime.control_max_us;
    record.control_overrun_count = frame.scheduler_runtime.control_overrun_count;
    record.control_skipped_count = frame.scheduler_runtime.control_skipped_count;
    record.output_last_us = frame.scheduler_runtime.output_last_us;
    record.output_max_us = frame.scheduler_runtime.output_max_us;
    record.output_overrun_count = frame.scheduler_runtime.output_overrun_count;
    record.output_skipped_count = frame.scheduler_runtime.output_skipped_count;
    record.flight_state = static_cast<std::uint8_t>(frame.flight_state);
    record.valid = frame.attitude.valid ? 1U : 0U;
    record.record_size = static_cast<std::uint16_t>(sizeof(BinaryLogRecord));
    return record;
}

} // namespace dfw::logging
