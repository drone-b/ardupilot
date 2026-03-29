#include "config/Parameters.hpp"

namespace dfw::config {

namespace {

constexpr float k_pi = 3.14159265358979323846f;

constexpr ParameterValue make_float(float value)
{
    ParameterValue parameter_value {};
    parameter_value.float32 = value;
    return parameter_value;
}

constexpr ParameterValue make_int(std::int32_t value)
{
    ParameterValue parameter_value {};
    parameter_value.int32 = value;
    return parameter_value;
}

constexpr ParameterValue make_bool(bool value)
{
    ParameterValue parameter_value {};
    parameter_value.boolean = value;
    return parameter_value;
}

} // namespace

ParameterRegistry& ParameterRegistry::instance()
{
    static ParameterRegistry registry {};
    return registry;
}

ParameterRegistry::ParameterRegistry() :
    parameters_ {
        Parameter {ParameterId::input_max_roll_angle_rad, "INPUT_MAX_ROLL", ParameterType::float32, make_float(0.523599f), make_float(0.523599f), make_float(0.0f), make_float(k_pi * 0.5f)},
        Parameter {ParameterId::input_max_pitch_angle_rad, "INPUT_MAX_PITCH", ParameterType::float32, make_float(0.523599f), make_float(0.523599f), make_float(0.0f), make_float(k_pi * 0.5f)},
        Parameter {ParameterId::input_max_yaw_rate_rad_s, "INPUT_MAX_YAW_RT", ParameterType::float32, make_float(2.5f), make_float(2.5f), make_float(0.0f), make_float(10.0f)},
        Parameter {ParameterId::attitude_roll_kp, "ATT_ROLL_KP", ParameterType::float32, make_float(4.0f), make_float(4.0f), make_float(0.0f), make_float(20.0f)},
        Parameter {ParameterId::attitude_pitch_kp, "ATT_PITCH_KP", ParameterType::float32, make_float(4.0f), make_float(4.0f), make_float(0.0f), make_float(20.0f)},
        Parameter {ParameterId::attitude_max_roll_rate_rad_s, "ATT_MAX_RRATE", ParameterType::float32, make_float(3.0f), make_float(3.0f), make_float(0.0f), make_float(20.0f)},
        Parameter {ParameterId::attitude_max_pitch_rate_rad_s, "ATT_MAX_PRATE", ParameterType::float32, make_float(3.0f), make_float(3.0f), make_float(0.0f), make_float(20.0f)},
        Parameter {ParameterId::rate_roll_kp, "RATE_ROLL_KP", ParameterType::float32, make_float(0.18f), make_float(0.18f), make_float(0.0f), make_float(5.0f)},
        Parameter {ParameterId::rate_roll_ki, "RATE_ROLL_KI", ParameterType::float32, make_float(0.08f), make_float(0.08f), make_float(0.0f), make_float(5.0f)},
        Parameter {ParameterId::rate_roll_kd, "RATE_ROLL_KD", ParameterType::float32, make_float(0.002f), make_float(0.002f), make_float(0.0f), make_float(1.0f)},
    }
{
}

bool ParameterRegistry::get_float(ParameterId id, float& out_value) const
{
    const Parameter* parameter = at(id);
    if (parameter == nullptr || parameter->type != ParameterType::float32) {
        return false;
    }

    out_value = parameter->value.float32;
    return true;
}

bool ParameterRegistry::get_int(ParameterId id, std::int32_t& out_value) const
{
    const Parameter* parameter = at(id);
    if (parameter == nullptr || parameter->type != ParameterType::int32) {
        return false;
    }

    out_value = parameter->value.int32;
    return true;
}

bool ParameterRegistry::get_bool(ParameterId id, bool& out_value) const
{
    const Parameter* parameter = at(id);
    if (parameter == nullptr || parameter->type != ParameterType::boolean) {
        return false;
    }

    out_value = parameter->value.boolean;
    return true;
}

bool ParameterRegistry::set_float(ParameterId id, float value)
{
    Parameter* parameter = at(id);
    if (parameter == nullptr || parameter->type != ParameterType::float32) {
        return false;
    }

    if (value < parameter->min_value.float32 || value > parameter->max_value.float32) {
        return false;
    }

    parameter->value.float32 = value;
    return true;
}

bool ParameterRegistry::set_int(ParameterId id, std::int32_t value)
{
    Parameter* parameter = at(id);
    if (parameter == nullptr || parameter->type != ParameterType::int32) {
        return false;
    }

    if (value < parameter->min_value.int32 || value > parameter->max_value.int32) {
        return false;
    }

    parameter->value.int32 = value;
    return true;
}

bool ParameterRegistry::set_bool(ParameterId id, bool value)
{
    Parameter* parameter = at(id);
    if (parameter == nullptr || parameter->type != ParameterType::boolean) {
        return false;
    }

    if (value != parameter->min_value.boolean && value != parameter->max_value.boolean) {
        return false;
    }

    parameter->value.boolean = value;
    return true;
}

void ParameterRegistry::reset_to_defaults()
{
    for (std::size_t i = 0; i < parameter_count_; ++i) {
        parameters_[i].value = parameters_[i].default_value;
    }
}

bool ParameterRegistry::load_from_storage()
{
    return false;
}

bool ParameterRegistry::save_to_storage() const
{
    return false;
}

std::uint16_t ParameterRegistry::schema_version() const
{
    return k_schema_version;
}

const Parameter* ParameterRegistry::parameter(ParameterId id) const
{
    return at(id);
}

Parameter* ParameterRegistry::at(ParameterId id)
{
    if (!valid_id(id)) {
        return nullptr;
    }

    return &parameters_[index_of(id)];
}

const Parameter* ParameterRegistry::at(ParameterId id) const
{
    if (!valid_id(id)) {
        return nullptr;
    }

    return &parameters_[index_of(id)];
}

} // namespace dfw::config
