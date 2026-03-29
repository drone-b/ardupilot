#pragma once

#include "common/TimeTypes.hpp"
#include "platform/Hal.hpp"

#include <cstdint>

namespace dfw::sensing {

enum class SensorKind {
    inertial,
    positioning,
    barometric,
    range,
    power,
    unknown
};

enum class ImuSampleStatus : std::uint8_t {
    ok = 0,
    no_data,
    invalid_timestamp,
    stale_sample,
    invalid_period,
    excessive_latency
};

struct MeasurementFrame {
    common::TimestampUs timestamp_us {};
    SensorKind kind {SensorKind::unknown};
    bool healthy {false};
};

class ISensor {
public:
    virtual ~ISensor() = default;

    virtual SensorKind kind() const = 0;
    virtual bool initialize() = 0;
    virtual bool sample(MeasurementFrame& out_frame) = 0;
};

struct ImuSample {
    common::TimestampUs sample_time_us {0};
    common::TimestampUs acquisition_time_us {0};
    common::DurationUs transport_latency_us {0};
    common::DurationUs sample_period_us {0};
    std::uint32_t sequence {0};
    float accel_mps2[3] {0.0f, 0.0f, 0.0f};
    float gyro_rad_s[3] {0.0f, 0.0f, 0.0f};
    float temperature_c {0.0f};
    bool valid {false};
    ImuSampleStatus status {ImuSampleStatus::no_data};
};

class IImuDevice {
public:
    virtual ~IImuDevice() = default;

    virtual bool initialize() = 0;
    virtual bool read_sample(common::TimestampUs trigger_time_us,
                             common::TimestampUs acquisition_time_us,
                             ImuSample& out_sample) = 0;
};

} // namespace dfw::sensing
