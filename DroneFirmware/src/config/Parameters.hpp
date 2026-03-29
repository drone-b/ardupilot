#pragma once

#include <cstddef>
#include <cstdint>

namespace dfw::config {

enum class ParameterType : std::uint8_t {
    float32 = 0,
    int32,
    boolean
};

enum class ParameterId : std::uint8_t {
    input_max_roll_angle_rad = 0,
    input_max_pitch_angle_rad,
    input_max_yaw_rate_rad_s,
    attitude_roll_kp,
    attitude_pitch_kp,
    attitude_max_roll_rate_rad_s,
    attitude_max_pitch_rate_rad_s,
    rate_roll_kp,
    rate_roll_ki,
    rate_roll_kd,
    count
};

union ParameterValue {
    float float32;
    std::int32_t int32;
    bool boolean;
};

struct Parameter {
    ParameterId id {};
    const char* name {nullptr};
    ParameterType type {ParameterType::float32};
    ParameterValue value {};
    ParameterValue default_value {};
    ParameterValue min_value {};
    ParameterValue max_value {};
};

class ParameterRegistry {
public:
    static constexpr std::uint16_t k_schema_version = 1;

    static ParameterRegistry& instance();

    bool get_float(ParameterId id, float& out_value) const;
    bool get_int(ParameterId id, std::int32_t& out_value) const;
    bool get_bool(ParameterId id, bool& out_value) const;

    bool set_float(ParameterId id, float value);
    bool set_int(ParameterId id, std::int32_t value);
    bool set_bool(ParameterId id, bool value);

    void reset_to_defaults();

    // Placeholder hooks for future non-volatile storage integration.
    bool load_from_storage();
    bool save_to_storage() const;
    std::uint16_t schema_version() const;
    const Parameter* parameter(ParameterId id) const;

private:
    ParameterRegistry();

    static constexpr std::size_t parameter_count_ =
        static_cast<std::size_t>(ParameterId::count);

    static constexpr std::size_t index_of(ParameterId id)
    {
        return static_cast<std::size_t>(id);
    }

    static constexpr bool valid_id(ParameterId id)
    {
        return index_of(id) < parameter_count_;
    }

    Parameter* at(ParameterId id);
    const Parameter* at(ParameterId id) const;

    Parameter parameters_[parameter_count_] {};
    bool dirty_ {false};
};

} // namespace dfw::config
