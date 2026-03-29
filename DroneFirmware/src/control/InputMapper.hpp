#pragma once

#include "common/TimeTypes.hpp"
#include "control/AttitudeController.hpp"
#include "config/Parameters.hpp"

namespace dfw::control {

enum class FlightMode : unsigned char {
    stabilize = 0,
    acro
};

struct PilotInput {
    float roll {-0.0f};
    float pitch {0.0f};
    float yaw {0.0f};
    float throttle {0.0f};
    bool valid {false};
};

struct PilotCommand {
    PilotInput input {};
    FlightMode mode {FlightMode::stabilize};
    bool arm {false};
    bool disarm {false};
    bool valid {false};
    common::TimestampUs timestamp_us {0};
};

class InputMapper {
public:
    InputMapper();

    AttitudeTarget map(const PilotInput& input) const;

private:
    static float clamp(float value, float min_value, float max_value);
};

} // namespace dfw::control
