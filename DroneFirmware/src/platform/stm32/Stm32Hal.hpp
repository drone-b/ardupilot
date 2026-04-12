#pragma once

#include "platform/BoardTarget.hpp"
#include "platform/Hal.hpp"
#include "platform/stm32/Stm32BusDevices.hpp"
#include <array>
#include <cstdint>

namespace dfw::platform::stm32 {

class IStm32Timebase {
public:
    virtual ~IStm32Timebase() = default;

    virtual common::TimestampUs now_us() const = 0;
    virtual void sleep_us(common::DurationUs duration_us) = 0;
};

class Stm32Clock final : public IClock {
public:
    explicit Stm32Clock(IStm32Timebase& timebase);

    common::TimestampUs now_us() const override;
    common::TimestampUs now_us_isr() const override;
    void sleep_us(common::DurationUs duration_us) override;

private:
    IStm32Timebase& timebase_;
};

class Stm32BoardControl final : public IBoardControl {
public:
    struct Stats {
        std::uint32_t watchdog_service_count {0};
        std::uint32_t reset_request_count {0};
    };

    void service_watchdog() override;
    void reset_system() override;
    const Stats& stats() const;
    bool reset_requested() const;

private:
    Stats stats_ {};
    bool reset_requested_ {false};
};

class Stm32OutputDriver final : public IOutputDriver {
public:
    void write_channel(std::uint8_t channel, float normalized_command) override;
};

class Stm32PwmBank final : public IPwmBank {
public:
    std::size_t channel_count() const override;
    Status configure_channel(std::uint8_t channel, const PwmChannelConfig& config) override;
    Status write_normalized(std::uint8_t channel, float normalized_command) override;
    void disarm_all() override;
    float channel_value(std::uint8_t channel) const;
    bool channel_configured(std::uint8_t channel) const;

private:
    static constexpr std::size_t k_pwm_channel_count = 8;

    std::array<PwmChannelConfig, k_pwm_channel_count> configs_ {};
    std::array<float, k_pwm_channel_count> values_ {};
    std::array<bool, k_pwm_channel_count> configured_ {};

    static float clamp_unit(float value);
};

class Stm32SensorInterruptRouter final : public ISensorInterruptRouter {
public:
    void register_data_ready_line(std::uint8_t line_index,
                                  void (*callback)(void*),
                                  void* context) override;
    bool simulate_data_ready(std::uint8_t line_index);

private:
    struct Route {
        void (*callback)(void*) {nullptr};
        void* context {nullptr};
        bool registered {false};
    };

    static constexpr std::size_t k_data_ready_line_count = 8;

    std::array<Route, k_data_ready_line_count> routes_ {};
};

class Stm32Hal final : public IHal {
public:
    Stm32Hal(BoardTargetProfile profile, IStm32Timebase& timebase);

    IClock& clock() override;
    IBoardControl& board_control() override;
    IOutputDriver& output_driver() override;
    IBusFactory& buses() override;
    IPwmBank& pwm_bank() override;
    ISensorInterruptRouter& sensor_interrupts() override;
    const BoardTargetProfile& profile() const;

private:
    BoardTargetProfile profile_ {};
    Stm32Clock clock_;
    Stm32BoardControl board_control_ {};
    Stm32OutputDriver output_driver_ {};
    Stm32BusFactory bus_factory_ {};
    Stm32PwmBank pwm_bank_ {};
    Stm32SensorInterruptRouter sensor_interrupt_router_ {};
};

} // namespace dfw::platform::stm32
