#include "control/RateController.hpp"

namespace dfw::control {

RateController::RateController(const RateGains& gains) :
    gains_(gains)
{
}

void RateController::reset()
{
    last_debug_ = {};
    integrator_ = 0.0f;
    filtered_derivative_ = 0.0f;
    previous_measured_rate_ = 0.0f;
    has_previous_measured_rate_ = false;
}

float RateController::update(float setpoint_rad_s, float measured_rad_s, float dt_s)
{
    const float error = setpoint_rad_s - measured_rad_s;
    const float proportional = gains_.kp * error;

    float derivative = 0.0f;
    if (dt_s > 0.0f) {
        if (has_previous_measured_rate_) {
            const float measured_rate_derivative =
                (measured_rad_s - previous_measured_rate_) / dt_s;
            const float alpha = clamp_unit(gains_.derivative_alpha);
            filtered_derivative_ +=
                alpha * (measured_rate_derivative - filtered_derivative_);
            derivative = -gains_.kd * filtered_derivative_;
        }
    }

    const float unsaturated_output = proportional + integrator_ + derivative;
    const float saturated_output = clamp(
        unsaturated_output,
        -gains_.output_limit,
        gains_.output_limit);
    last_debug_.error = error;
    last_debug_.proportional = proportional;
    last_debug_.derivative = derivative;
    last_debug_.unsaturated_output = unsaturated_output;
    last_debug_.saturated_output = saturated_output;
    last_debug_.saturated = saturated_output != unsaturated_output;

    if (dt_s > 0.0f && gains_.ki > 0.0f) {
        const bool saturated_high = saturated_output >= gains_.output_limit && error > 0.0f;
        const bool saturated_low = saturated_output <= -gains_.output_limit && error < 0.0f;

        if (!saturated_high && !saturated_low) {
            integrator_ += error * dt_s * gains_.ki;
            integrator_ = clamp(integrator_, -gains_.integrator_limit, gains_.integrator_limit);
        }
    }

    last_debug_.integrator = integrator_;

    previous_measured_rate_ = measured_rad_s;
    has_previous_measured_rate_ = true;

    return saturated_output;
}

const RateControllerDebug& RateController::last_debug() const
{
    return last_debug_;
}

float RateController::clamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

float RateController::clamp_unit(float value)
{
    return clamp(value, 0.0f, 1.0f);
}

} // namespace dfw::control
