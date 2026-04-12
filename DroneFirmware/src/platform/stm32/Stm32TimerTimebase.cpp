#include "platform/stm32/Stm32TimerTimebase.hpp"

namespace dfw::platform::stm32 {

Stm32TimerTimebase::Stm32TimerTimebase(std::uint32_t timer_hz) :
    timer_hz_(timer_hz == 0U ? 1000000U : timer_hz),
    tick_period_us_(static_cast<common::DurationUs>(1000000U / timer_hz_))
{
    if (tick_period_us_ == 0U) {
        tick_period_us_ = 1U;
    }
}

common::TimestampUs Stm32TimerTimebase::now_us() const
{
    return now_us_;
}

void Stm32TimerTimebase::sleep_us(common::DurationUs duration_us)
{
    now_us_ += duration_us;
}

void Stm32TimerTimebase::on_timer_tick()
{
    now_us_ += tick_period_us_;
}

std::uint32_t Stm32TimerTimebase::timer_hz() const
{
    return timer_hz_;
}

} // namespace dfw::platform::stm32
