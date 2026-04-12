#include "platform/BoardTarget.hpp"

namespace dfw::platform {

BoardTargetProfile BoardTargetRegistry::simulation_profile()
{
    BoardTargetProfile profile {};
    profile.family = BoardFamily::simulation;
    profile.name = "sim";
    profile.dma_spi = false;
    profile.dma_uart = false;
    profile.can_bus = false;
    profile.sensor_irq_router = true;
    profile.pwm_channels = 8;
    return profile;
}

BoardTargetProfile BoardTargetRegistry::stm32h743vit_usb_profile()
{
    BoardTargetProfile profile {};
    profile.family = BoardFamily::stm32h7;
    profile.name = "stm32h743vit_usb";
    profile.dma_spi = false;
    profile.dma_uart = false;
    profile.can_bus = false;
    profile.sensor_irq_router = false;
    profile.pwm_channels = 0;
    return profile;
}

BoardTargetProfile BoardTargetRegistry::stm32h743vit_reference_profile()
{
    BoardTargetProfile profile {};
    profile.family = BoardFamily::stm32h7;
    profile.name = "stm32h743vit_reference";
    profile.dma_spi = false;
    profile.dma_uart = false;
    profile.can_bus = false;
    profile.sensor_irq_router = true;
    profile.pwm_channels = 8;
    return profile;
}

bool BoardTargetRegistry::flight_qualification_ready(const BoardTargetProfile& profile)
{
    const bool dma_ready = profile.dma_spi && profile.dma_uart;
    const bool io_ready = profile.can_bus && profile.sensor_irq_router;
    const bool actuator_ready = profile.pwm_channels >= 4U;
    return dma_ready && io_ready && actuator_ready;
}

} // namespace dfw::platform
