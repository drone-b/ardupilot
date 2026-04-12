#include "sensing/PowerMonitor.hpp"

namespace dfw::sensing {

PowerMonitor::PowerMonitor(platform::IAnalogInput& voltage_input,
                           platform::IAnalogInput& current_input,
                           const PowerMonitorConfig& config) :
    voltage_input_(voltage_input),
    current_input_(current_input),
    config_(config)
{
}

bool PowerMonitor::sample(common::TimestampUs now_us, PowerSample& out_sample)
{
    PowerSample sample {};
    sample.sample_time_us = now_us;
    sample.sequence = next_sequence_++;

    if (now_us == 0U) {
        sample.status = PowerSampleStatus::invalid_timestamp;
        latest_ = sample;
        out_sample = sample;
        return false;
    }

    float voltage_adc_v = 0.0f;
    if (!voltage_input_.read_voltage(voltage_adc_v).success()) {
        sample.status = PowerSampleStatus::no_data;
        latest_ = sample;
        out_sample = sample;
        return false;
    }

    float current_adc_v = 0.0f;
    if (!current_input_.read_voltage(current_adc_v).success()) {
        sample.status = PowerSampleStatus::no_data;
        latest_ = sample;
        out_sample = sample;
        return false;
    }

    sample.voltage_v = voltage_adc_v * config_.voltage_scale;
    sample.current_a = (current_adc_v - config_.current_offset_v) * config_.current_scale;

    if (sample.voltage_v < config_.min_valid_voltage_v ||
        sample.voltage_v > config_.max_valid_voltage_v) {
        sample.status = PowerSampleStatus::invalid_voltage;
        latest_ = sample;
        out_sample = sample;
        return false;
    }

    if (sample.current_a < config_.min_valid_current_a ||
        sample.current_a > config_.max_valid_current_a) {
        sample.status = PowerSampleStatus::invalid_current;
        latest_ = sample;
        out_sample = sample;
        return false;
    }

    const float voltage_span = config_.full_voltage_v - config_.empty_voltage_v;
    if (voltage_span > 0.0f) {
        sample.remaining_ratio =
            clamp((sample.voltage_v - config_.empty_voltage_v) / voltage_span, 0.0f, 1.0f);
    }

    sample.valid = true;
    sample.status = PowerSampleStatus::ok;
    latest_ = sample;
    out_sample = sample;
    return true;
}

const PowerSample& PowerMonitor::latest() const
{
    return latest_;
}

float PowerMonitor::clamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

} // namespace dfw::sensing
