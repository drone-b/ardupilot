#include "comms/AiStatePublisher.hpp"

#include <cstring>

namespace dfw::comms {

AiStatePublisher::AiStatePublisher(platform::IUartPort& uart_port) :
    uart_port_(uart_port)
{
}

void AiStatePublisher::publish(const AiStateSnapshotV1& snapshot)
{
    const FramedPacket packet = make_packet(snapshot, sequence_counter_);
    ++sequence_counter_;
    (void) uart_port_.write(reinterpret_cast<const std::uint8_t*>(&packet), sizeof(packet));
}

AiStatePublisher::BinarySnapshotPayload
AiStatePublisher::make_payload(const AiStateSnapshotV1& snapshot, std::uint32_t sequence)
{
    BinarySnapshotPayload payload {};
    write_le32(payload.magic_le, 0x44534149U);
    write_le16(payload.version_le, 1U);
    write_le16(payload.payload_size_le, static_cast<std::uint16_t>(sizeof(BinarySnapshotPayload)));
    write_le32(payload.sequence_le, sequence);
    write_le64(payload.timestamp_us_le, static_cast<std::uint64_t>(snapshot.timestamp_us));

    for (std::uint8_t i = 0; i < 4U; ++i) {
        write_float_le(payload.q_le[i], snapshot.q[i]);
        write_float_le(payload.alloc_unallocated_le[i], snapshot.alloc_unallocated[i]);
    }

    for (std::uint8_t i = 0; i < 3U; ++i) {
        write_float_le(payload.omega_body_le[i], snapshot.omega_body[i]);
        write_float_le(payload.accel_body_le[i], snapshot.accel_body[i]);
        write_float_le(payload.gyro_bias_le[i], snapshot.gyro_bias[i]);
        write_float_le(payload.velocity_ned_le[i], snapshot.velocity_ned[i]);
        write_float_le(payload.position_ned_le[i], snapshot.position_ned[i]);
        write_float_le(payload.innovation_norm_le[i], snapshot.innovation_norm[i]);
    }

    payload.gate_status = snapshot.gate_status;
    payload.estimator_lane = snapshot.estimator_lane;
    payload.estimator_health = snapshot.estimator_health;
    payload.scheduler_mode = snapshot.scheduler_mode;
    write_le32(payload.alloc_saturated_mask_le, snapshot.alloc_saturated_mask);
    write_le32(payload.safety_state_le, snapshot.safety_state);
    write_le32(payload.arming_state_le, snapshot.arming_state);
    payload.valid = snapshot.valid ? 1U : 0U;
    return payload;
}

AiStatePublisher::FramedPacket
AiStatePublisher::make_packet(const AiStateSnapshotV1& snapshot, std::uint32_t sequence)
{
    FramedPacket packet {};
    packet.payload = make_payload(snapshot, sequence);
    write_le16(packet.header.sync_word_le, 0xAA5AU);
    write_le16(packet.header.payload_size_le,
               static_cast<std::uint16_t>(sizeof(packet.payload)));
    write_le32(packet.header.crc32_le, compute_crc32(packet.payload));
    return packet;
}

std::uint32_t AiStatePublisher::compute_crc32(const BinarySnapshotPayload& payload)
{
    const auto* bytes = reinterpret_cast<const std::uint8_t*>(&payload);
    std::uint32_t crc = 0xFFFFFFFFU;

    for (std::size_t index = 0; index < sizeof(BinarySnapshotPayload); ++index) {
        crc ^= static_cast<std::uint32_t>(bytes[index]);
        for (std::uint8_t bit = 0; bit < 8U; ++bit) {
            const std::uint32_t mask = static_cast<std::uint32_t>(-(crc & 1U));
            crc = (crc >> 1U) ^ (0xEDB88320U & mask);
        }
    }

    return ~crc;
}

void AiStatePublisher::write_le16(std::uint8_t* destination, std::uint16_t value)
{
    destination[0] = static_cast<std::uint8_t>(value & 0xFFU);
    destination[1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
}

void AiStatePublisher::write_le32(std::uint8_t* destination, std::uint32_t value)
{
    destination[0] = static_cast<std::uint8_t>(value & 0xFFU);
    destination[1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
    destination[2] = static_cast<std::uint8_t>((value >> 16U) & 0xFFU);
    destination[3] = static_cast<std::uint8_t>((value >> 24U) & 0xFFU);
}

void AiStatePublisher::write_le64(std::uint8_t* destination, std::uint64_t value)
{
    for (std::uint8_t index = 0; index < 8U; ++index) {
        destination[index] = static_cast<std::uint8_t>((value >> (index * 8U)) & 0xFFU);
    }
}

void AiStatePublisher::write_float_le(std::uint8_t* destination, float value)
{
    std::uint32_t bits = 0U;
    static_assert(sizeof(bits) == sizeof(value), "Float serialization size mismatch");
    std::memcpy(&bits, &value, sizeof(bits));
    write_le32(destination, bits);
}

} // namespace dfw::comms
