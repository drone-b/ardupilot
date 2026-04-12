#pragma once

#include "common/TimeTypes.hpp"
#include "platform/Hal.hpp"

#include <cstdint>

namespace dfw::sensing {

enum class PowerSampleStatus : std::uint8_t {
    ok = 0,
    no_data,
    invalid_timestamp,
    invalid_voltage,
    invalid_current
};

struct PowerMonitorConfig {
    float voltage_scale {1.0f};
    float current_scale {1.0f};
    float current_offset_v {0.0f};
    float empty_voltage_v {10.5f};
    float full_voltage_v {12.6f};
    float min_valid_voltage_v {0.0f};
    float max_valid_voltage_v {60.0f};
    float min_valid_current_a {-5.0f};
    float max_valid_current_a {200.0f};
};

struct PowerSample {
    common::TimestampUs sample_time_us {0};
    std::uint32_t sequence {0};
    float voltage_v {0.0f};
    float current_a {0.0f};
    float remaining_ratio {0.0f};
    bool valid {false};
    PowerSampleStatus status {PowerSampleStatus::no_data};
};

class PowerMonitor {
public:
    PowerMonitor(platform::IAnalogInput& voltage_input,
                 platform::IAnalogInput& current_input,
                 const PowerMonitorConfig& config);

    bool sample(common::TimestampUs now_us, PowerSample& out_sample);
    const PowerSample& latest() const;

private:
    static float clamp(float value, float min_value, float max_value);

    platform::IAnalogInput& voltage_input_;
    platform::IAnalogInput& current_input_;
    PowerMonitorConfig config_ {};
    PowerSample latest_ {};
    std::uint32_t next_sequence_ {1};
};

} // namespace dfw::sensing
