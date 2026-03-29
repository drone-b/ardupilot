#pragma once

#include "common/TimeTypes.hpp"
#include "platform/Hal.hpp"

#include <cstdint>

namespace dfw::comms {

struct AiStateSnapshotV1 {
    common::TimestampUs timestamp_us {0};
    float q[4] {1.0f, 0.0f, 0.0f, 0.0f};
    float omega_body[3] {0.0f, 0.0f, 0.0f};
    float accel_body[3] {0.0f, 0.0f, 0.0f};
    float gyro_bias[3] {0.0f, 0.0f, 0.0f};
    float velocity_ned[3] {0.0f, 0.0f, 0.0f};
    float position_ned[3] {0.0f, 0.0f, 0.0f};
    float innovation_norm[3] {0.0f, 0.0f, 0.0f};
    std::uint8_t gate_status {0};
    std::uint8_t estimator_lane {0};
    std::uint8_t estimator_health {2};
    std::uint8_t scheduler_mode {0};
    std::uint32_t alloc_saturated_mask {0};
    float alloc_unallocated[4] {0.0f, 0.0f, 0.0f, 0.0f};
    std::uint32_t safety_state {0};
    std::uint32_t arming_state {0};
    bool valid {false};
};

class AiStatePublisher {
public:
    explicit AiStatePublisher(platform::IUartPort& uart_port);

    void publish(const AiStateSnapshotV1& snapshot);

private:
    struct __attribute__((packed)) BinarySnapshotPayload {
        std::uint8_t magic_le[4] {0, 0, 0, 0};
        std::uint8_t version_le[2] {0, 0};
        std::uint8_t payload_size_le[2] {0, 0};
        std::uint8_t sequence_le[4] {0, 0, 0, 0};
        std::uint8_t timestamp_us_le[8] {0, 0, 0, 0, 0, 0, 0, 0};
        std::uint8_t q_le[4][4] {};
        std::uint8_t omega_body_le[3][4] {};
        std::uint8_t accel_body_le[3][4] {};
        std::uint8_t gyro_bias_le[3][4] {};
        std::uint8_t velocity_ned_le[3][4] {};
        std::uint8_t position_ned_le[3][4] {};
        std::uint8_t innovation_norm_le[3][4] {};
        std::uint8_t gate_status {0};
        std::uint8_t estimator_lane {0};
        std::uint8_t estimator_health {0};
        std::uint8_t scheduler_mode {0};
        std::uint8_t alloc_saturated_mask_le[4] {0, 0, 0, 0};
        std::uint8_t alloc_unallocated_le[4][4] {};
        std::uint8_t safety_state_le[4] {0, 0, 0, 0};
        std::uint8_t arming_state_le[4] {0, 0, 0, 0};
        std::uint8_t valid {0};
        std::uint8_t reserved[3] {0, 0, 0};
    };

    struct __attribute__((packed)) BinaryFrameHeader {
        std::uint8_t sync_word_le[2] {0, 0};
        std::uint8_t payload_size_le[2] {0, 0};
        std::uint8_t crc32_le[4] {0, 0, 0, 0};
    };

    struct __attribute__((packed)) FramedPacket {
        BinaryFrameHeader header {};
        BinarySnapshotPayload payload {};
    };

    static BinarySnapshotPayload make_payload(const AiStateSnapshotV1& snapshot,
                                              std::uint32_t sequence);
    static FramedPacket make_packet(const AiStateSnapshotV1& snapshot,
                                    std::uint32_t sequence);
    static std::uint32_t compute_crc32(const BinarySnapshotPayload& payload);
    static void write_le16(std::uint8_t* destination, std::uint16_t value);
    static void write_le32(std::uint8_t* destination, std::uint32_t value);
    static void write_le64(std::uint8_t* destination, std::uint64_t value);
    static void write_float_le(std::uint8_t* destination, float value);

    platform::IUartPort& uart_port_;
    std::uint32_t sequence_counter_ {0};
};

} // namespace dfw::comms
