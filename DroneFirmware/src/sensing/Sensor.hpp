#pragma once

#include "common/TimeTypes.hpp"
#include "platform/Hal.hpp"

#include <cstdint>

namespace dfw::sensing {

enum class SensorKind {
    inertial,
    positioning,
    barometric,
    magnetic,
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

enum class BarometerSampleStatus : std::uint8_t {
    ok = 0,
    no_data,
    invalid_timestamp,
    invalid_pressure,
    invalid_temperature
};

enum class MagnetometerSampleStatus : std::uint8_t {
    ok = 0,
    no_data,
    invalid_timestamp,
    invalid_field,
    invalid_temperature
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

struct ImuCalibration {
    float accel_bias_mps2[3] {0.0f, 0.0f, 0.0f};
    float gyro_bias_rad_s[3] {0.0f, 0.0f, 0.0f};
    float accel_scale[3] {1.0f, 1.0f, 1.0f};
    float gyro_scale[3] {1.0f, 1.0f, 1.0f};
    bool valid {true};
};

struct SensorHealth {
    common::TimestampUs last_update_us {0};
    std::uint32_t total_sample_count {0};
    std::uint32_t total_error_count {0};
    std::uint32_t consecutive_error_count {0};
    bool initialized {false};
    bool healthy {false};
    ImuSampleStatus last_status {ImuSampleStatus::no_data};
};

struct BarometerSample {
    common::TimestampUs sample_time_us {0};
    common::TimestampUs acquisition_time_us {0};
    common::DurationUs transport_latency_us {0};
    std::uint32_t sequence {0};
    float pressure_pa {0.0f};
    float temperature_c {0.0f};
    float altitude_m {0.0f};
    bool valid {false};
    BarometerSampleStatus status {BarometerSampleStatus::no_data};
};

struct MagnetometerSample {
    common::TimestampUs sample_time_us {0};
    common::TimestampUs acquisition_time_us {0};
    common::DurationUs transport_latency_us {0};
    std::uint32_t sequence {0};
    float magnetic_field_ut[3] {0.0f, 0.0f, 0.0f};
    float temperature_c {0.0f};
    bool valid {false};
    MagnetometerSampleStatus status {MagnetometerSampleStatus::no_data};
};

class IImuDevice {
public:
    virtual ~IImuDevice() = default;

    virtual bool initialize() = 0;
    virtual bool read_sample(common::TimestampUs trigger_time_us,
                             common::TimestampUs acquisition_time_us,
                             ImuSample& out_sample) = 0;
};

class IBarometerDevice {
public:
    virtual ~IBarometerDevice() = default;

    virtual bool initialize() = 0;
    virtual bool read_sample(common::TimestampUs trigger_time_us,
                             common::TimestampUs acquisition_time_us,
                             BarometerSample& out_sample) = 0;
};

class IMagnetometerDevice {
public:
    virtual ~IMagnetometerDevice() = default;

    virtual bool initialize() = 0;
    virtual bool read_sample(common::TimestampUs trigger_time_us,
                             common::TimestampUs acquisition_time_us,
                             MagnetometerSample& out_sample) = 0;
};

} // namespace dfw::sensing
