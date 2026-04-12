#include "platform/stm32/Stm32Sim.hpp"

namespace dfw::platform::stm32 {

common::TimestampUs SimulatedStm32Timebase::now_us() const
{
    return now_us_;
}

void SimulatedStm32Timebase::sleep_us(common::DurationUs duration_us)
{
    advance_us(duration_us);
}

void SimulatedStm32Timebase::advance_us(common::DurationUs duration_us)
{
    now_us_ += duration_us;
}

} // namespace dfw::platform::stm32
