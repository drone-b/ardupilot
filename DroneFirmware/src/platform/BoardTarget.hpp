#pragma once

#include <cstdint>

namespace dfw::platform {

enum class BoardFamily : std::uint8_t {
    simulation = 0,
    stm32f4,
    stm32h7,
};

struct BoardTargetProfile {
    BoardFamily family {BoardFamily::simulation};
    const char* name {"sim"};
    bool dma_spi {false};
    bool dma_uart {false};
    bool can_bus {false};
    bool sensor_irq_router {true};
    std::uint8_t pwm_channels {0};
};

class BoardTargetRegistry {
public:
    static BoardTargetProfile simulation_profile();
    static BoardTargetProfile stm32h743vit_usb_profile();
    static BoardTargetProfile stm32h743vit_reference_profile();
    static bool flight_qualification_ready(const BoardTargetProfile& profile);
};

} // namespace dfw::platform
