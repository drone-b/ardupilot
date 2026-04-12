#include "platform/stm32/Stm32Hal.hpp"

namespace dfw::platform::stm32 {

Stm32Clock::Stm32Clock(IStm32Timebase& timebase) :
    timebase_(timebase)
{
}

common::TimestampUs Stm32Clock::now_us() const
{
    return timebase_.now_us();
}

common::TimestampUs Stm32Clock::now_us_isr() const
{
    return now_us();
}

void Stm32Clock::sleep_us(common::DurationUs duration_us)
{
    timebase_.sleep_us(duration_us);
}

void Stm32BoardControl::service_watchdog()
{
    ++stats_.watchdog_service_count;
}

void Stm32BoardControl::reset_system()
{
    reset_requested_ = true;
    ++stats_.reset_request_count;
}

const Stm32BoardControl::Stats& Stm32BoardControl::stats() const
{
    return stats_;
}

bool Stm32BoardControl::reset_requested() const
{
    return reset_requested_;
}

void Stm32OutputDriver::write_channel(std::uint8_t channel, float normalized_command)
{
    (void) channel;
    (void) normalized_command;
}

std::size_t Stm32PwmBank::channel_count() const
{
    return k_pwm_channel_count;
}

Status Stm32PwmBank::configure_channel(std::uint8_t channel, const PwmChannelConfig& config)
{
    if (channel >= k_pwm_channel_count) {
        return Status {StatusCode::invalid_argument};
    }

    configs_[channel] = config;
    configured_[channel] = true;
    values_[channel] = 0.0f;
    return {};
}

Status Stm32PwmBank::write_normalized(std::uint8_t channel, float normalized_command)
{
    if (channel >= k_pwm_channel_count || !configured_[channel]) {
        return Status {StatusCode::invalid_argument};
    }

    values_[channel] = clamp_unit(normalized_command);
    return {};
}

void Stm32PwmBank::disarm_all()
{
    for (float& value : values_) {
        value = 0.0f;
    }
}

float Stm32PwmBank::channel_value(std::uint8_t channel) const
{
    if (channel >= k_pwm_channel_count) {
        return 0.0f;
    }

    return values_[channel];
}

bool Stm32PwmBank::channel_configured(std::uint8_t channel) const
{
    if (channel >= k_pwm_channel_count) {
        return false;
    }

    return configured_[channel];
}

float Stm32PwmBank::clamp_unit(float value)
{
    if (value < 0.0f) {
        return 0.0f;
    }

    if (value > 1.0f) {
        return 1.0f;
    }

    return value;
}

void Stm32SensorInterruptRouter::register_data_ready_line(std::uint8_t line_index,
                                                          void (*callback)(void*),
                                                          void* context)
{
    if (line_index >= k_data_ready_line_count) {
        return;
    }

    routes_[line_index].callback = callback;
    routes_[line_index].context = context;
    routes_[line_index].registered = callback != nullptr;
}

bool Stm32SensorInterruptRouter::simulate_data_ready(std::uint8_t line_index)
{
    if (line_index >= k_data_ready_line_count ||
        !routes_[line_index].registered ||
        routes_[line_index].callback == nullptr) {
        return false;
    }

    routes_[line_index].callback(routes_[line_index].context);
    return true;
}

Stm32Hal::Stm32Hal(BoardTargetProfile profile, IStm32Timebase& timebase) :
    profile_(profile),
    clock_(timebase)
{
}

IClock& Stm32Hal::clock()
{
    return clock_;
}

IBoardControl& Stm32Hal::board_control()
{
    return board_control_;
}

IOutputDriver& Stm32Hal::output_driver()
{
    return output_driver_;
}

IBusFactory& Stm32Hal::buses()
{
    return bus_factory_;
}

IPwmBank& Stm32Hal::pwm_bank()
{
    return pwm_bank_;
}

ISensorInterruptRouter& Stm32Hal::sensor_interrupts()
{
    return sensor_interrupt_router_;
}

const BoardTargetProfile& Stm32Hal::profile() const
{
    return profile_;
}

} // namespace dfw::platform::stm32
