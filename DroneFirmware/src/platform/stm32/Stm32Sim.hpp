#pragma once

#include "platform/stm32/Stm32Hal.hpp"

namespace dfw::platform::stm32 {

class SimulatedStm32Timebase final : public IStm32Timebase {
public:
    common::TimestampUs now_us() const override;
    void sleep_us(common::DurationUs duration_us) override;
    void advance_us(common::DurationUs duration_us);

private:
    common::TimestampUs now_us_ {0};
};

} // namespace dfw::platform::stm32
