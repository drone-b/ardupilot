#pragma once

#include "platform/stm32/Stm32Hal.hpp"

namespace dfw::platform::stm32 {

class Stm32TimerTimebase final : public IStm32Timebase {
public:
    explicit Stm32TimerTimebase(std::uint32_t timer_hz);

    common::TimestampUs now_us() const override;
    void sleep_us(common::DurationUs duration_us) override;
    void on_timer_tick();
    std::uint32_t timer_hz() const;

private:
    std::uint32_t timer_hz_ {1000000U};
    common::DurationUs tick_period_us_ {1U};
    common::TimestampUs now_us_ {0};
};

} // namespace dfw::platform::stm32
