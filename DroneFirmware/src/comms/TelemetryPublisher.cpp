#include "comms/TelemetryPublisher.hpp"

#include <cstring>

namespace dfw::comms {

TelemetryPublisher::TelemetryPublisher(platform::IUartPort& uart_port) :
    uart_port_(uart_port)
{
}

void TelemetryPublisher::publish(const TelemetryFrame& frame)
{
    const FramedTelemetryPacket packet = make_framed_packet(frame, sequence_counter_);
    ++sequence_counter_;
    (void) uart_port_.write(reinterpret_cast<const std::uint8_t*>(&packet), sizeof(packet));
}

TelemetryPublisher::BinaryTelemetryFrame
TelemetryPublisher::make_binary_frame(const TelemetryFrame& frame, std::uint32_t sequence)
{
    static_assert(sizeof(BinaryTelemetryFrame) == 156,
                  "Binary telemetry frame size must remain fixed");

    BinaryTelemetryFrame packet {};
    write_le32(packet.magic_le, 0x44544657U);
    write_le16(packet.version_le, 1U);
    write_le16(packet.frame_size_le, static_cast<std::uint16_t>(sizeof(BinaryTelemetryFrame)));
    write_le32(packet.sequence_le, sequence);
    write_le64(packet.timestamp_us_le, static_cast<std::uint64_t>(frame.timestamp_us));
    write_float_le(packet.attitude_rad_le[0], frame.attitude.roll_rad);
    write_float_le(packet.attitude_rad_le[1], frame.attitude.pitch_rad);
    write_float_le(packet.attitude_rad_le[2], frame.attitude.yaw_rad);
    write_float_le(packet.gyro_rad_s_le[0], frame.imu_sample.gyro_rad_s[0]);
    write_float_le(packet.gyro_rad_s_le[1], frame.imu_sample.gyro_rad_s[1]);
    write_float_le(packet.gyro_rad_s_le[2], frame.imu_sample.gyro_rad_s[2]);
    write_float_le(packet.control_le[0], frame.control_demand.roll);
    write_float_le(packet.control_le[1], frame.control_demand.pitch);
    write_float_le(packet.control_le[2], frame.control_demand.yaw);
    write_float_le(packet.control_le[3], frame.control_demand.thrust);
    write_float_le(packet.rate_error_rad_s_le[0], frame.control_debug.roll.rate_error_rad_s);
    write_float_le(packet.rate_error_rad_s_le[1], frame.control_debug.pitch.rate_error_rad_s);
    write_float_le(packet.rate_error_rad_s_le[2], frame.control_debug.yaw.rate_error_rad_s);
    write_float_le(packet.pid_p_le[0], frame.control_debug.roll.proportional);
    write_float_le(packet.pid_p_le[1], frame.control_debug.pitch.proportional);
    write_float_le(packet.pid_p_le[2], frame.control_debug.yaw.proportional);
    write_float_le(packet.pid_i_le[0], frame.control_debug.roll.integrator);
    write_float_le(packet.pid_i_le[1], frame.control_debug.pitch.integrator);
    write_float_le(packet.pid_i_le[2], frame.control_debug.yaw.integrator);
    write_float_le(packet.pid_d_le[0], frame.control_debug.roll.derivative);
    write_float_le(packet.pid_d_le[1], frame.control_debug.pitch.derivative);
    write_float_le(packet.pid_d_le[2], frame.control_debug.yaw.derivative);
    write_float_le(packet.unsaturated_output_le[0], frame.control_debug.roll.unsaturated_output);
    write_float_le(packet.unsaturated_output_le[1], frame.control_debug.pitch.unsaturated_output);
    write_float_le(packet.unsaturated_output_le[2], frame.control_debug.yaw.unsaturated_output);
    write_float_le(packet.saturated_output_le[0], frame.control_debug.roll.saturated_output);
    write_float_le(packet.saturated_output_le[1], frame.control_debug.pitch.saturated_output);
    write_float_le(packet.saturated_output_le[2], frame.control_debug.yaw.saturated_output);
    packet.saturation_flags =
        (frame.control_debug.roll.saturated ? 0x01U : 0U) |
        (frame.control_debug.pitch.saturated ? 0x02U : 0U) |
        (frame.control_debug.yaw.saturated ? 0x04U : 0U);
    packet.debug_reserved[0] =
        static_cast<std::uint8_t>(frame.allocator_status.saturated_mask & 0xFFU);
    packet.debug_reserved[1] =
        static_cast<std::uint8_t>(frame.allocator_status.condition_warning & 0xFFU);
    packet.debug_reserved[2] =
        static_cast<std::uint8_t>(frame.allocator_status.solve_iterations & 0xFFU);
    write_float_le(packet.motors_le[0], frame.motor_outputs.values[0]);
    write_float_le(packet.motors_le[1], frame.motor_outputs.values[1]);
    write_float_le(packet.motors_le[2], frame.motor_outputs.values[2]);
    write_float_le(packet.motors_le[3], frame.motor_outputs.values[3]);
    packet.flight_state = static_cast<std::uint8_t>(frame.flight_state);
    packet.reserved[0] = frame.scheduler_mode;
    packet.reserved[1] = frame.ai_link_fresh;
    packet.reserved[2] = frame.authority_fallback_reason;
    return packet;
}

TelemetryPublisher::FramedTelemetryPacket
TelemetryPublisher::make_framed_packet(const TelemetryFrame& frame, std::uint32_t sequence)
{
    static_assert(sizeof(BinaryTelemetryHeader) == 8,
                  "Telemetry header size must remain fixed");
    static_assert(sizeof(BinaryTelemetryFrame) == 156,
                  "Binary telemetry frame size must remain fixed");
    static_assert(sizeof(FramedTelemetryPacket) == 164,
                  "Framed telemetry packet size must remain fixed");

    FramedTelemetryPacket packet {};
    packet.payload = make_binary_frame(frame, sequence);
    write_le16(packet.header.sync_word_le, 0xAA55U);
    write_le16(packet.header.payload_size_le,
               static_cast<std::uint16_t>(sizeof(packet.payload)));
    write_le32(packet.header.crc32_le, compute_crc32(packet.payload));
    return packet;
}

std::uint32_t TelemetryPublisher::compute_crc32(const BinaryTelemetryFrame& payload)
{
    const auto* bytes = reinterpret_cast<const std::uint8_t*>(&payload);
    std::uint32_t crc = 0xFFFFFFFFU;

    for (std::size_t index = 0; index < sizeof(BinaryTelemetryFrame); ++index) {
        crc ^= static_cast<std::uint32_t>(bytes[index]);
        for (std::uint8_t bit = 0; bit < 8U; ++bit) {
            const std::uint32_t mask = static_cast<std::uint32_t>(-(crc & 1U));
            crc = (crc >> 1U) ^ (0xEDB88320U & mask);
        }
    }

    return ~crc;
}

void TelemetryPublisher::write_le16(std::uint8_t* destination, std::uint16_t value)
{
    destination[0] = static_cast<std::uint8_t>(value & 0xFFU);
    destination[1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
}

void TelemetryPublisher::write_le32(std::uint8_t* destination, std::uint32_t value)
{
    destination[0] = static_cast<std::uint8_t>(value & 0xFFU);
    destination[1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
    destination[2] = static_cast<std::uint8_t>((value >> 16U) & 0xFFU);
    destination[3] = static_cast<std::uint8_t>((value >> 24U) & 0xFFU);
}

void TelemetryPublisher::write_le64(std::uint8_t* destination, std::uint64_t value)
{
    for (std::uint8_t index = 0; index < 8U; ++index) {
        destination[index] = static_cast<std::uint8_t>((value >> (index * 8U)) & 0xFFU);
    }
}

void TelemetryPublisher::write_float_le(std::uint8_t* destination, float value)
{
    std::uint32_t bits = 0U;
    static_assert(sizeof(bits) == sizeof(value), "Float serialization size mismatch");
    std::memcpy(&bits, &value, sizeof(bits));
    write_le32(destination, bits);
}

} // namespace dfw::comms
