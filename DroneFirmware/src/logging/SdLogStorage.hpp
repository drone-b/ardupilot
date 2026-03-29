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
        std::uint16_t version {1};
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
        std::uint8_t flight_state {0};
        std::uint8_t valid {0};
        std::uint16_t record_size {0};
    };

    static BinaryLogRecord make_record(const comms::TelemetryFrame& frame);

    IAppendStorageDevice& device_;
    bool initialized_ {false};
    bool healthy_ {true};
};

} // namespace dfw::logging
