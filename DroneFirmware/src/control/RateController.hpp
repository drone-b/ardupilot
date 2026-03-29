#pragma once

namespace dfw::control {

struct RateGains {
    float kp {0.0f};
    float ki {0.0f};
    float kd {0.0f};
    float integrator_limit {0.0f};
    float output_limit {1.0f};
    float derivative_alpha {0.2f};
};

struct RateControllerDebug {
    float error {0.0f};
    float proportional {0.0f};
    float integrator {0.0f};
    float derivative {0.0f};
    float unsaturated_output {0.0f};
    float saturated_output {0.0f};
    bool saturated {false};
};

class RateController {
public:
    explicit RateController(const RateGains& gains);

    void reset();
    float update(float setpoint_rad_s, float measured_rad_s, float dt_s);
    const RateControllerDebug& last_debug() const;

private:
    static float clamp(float value, float min_value, float max_value);
    static float clamp_unit(float value);

    RateGains gains_ {};
    RateControllerDebug last_debug_ {};
    float integrator_ {0.0f};
    float filtered_derivative_ {0.0f};
    float previous_measured_rate_ {0.0f};
    bool has_previous_measured_rate_ {false};
};

} // namespace dfw::control
