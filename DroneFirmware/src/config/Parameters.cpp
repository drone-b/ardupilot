#include "config/Parameters.hpp"

#include <cstdio>

namespace dfw::config {

namespace {

constexpr std::uint32_t k_storage_magic = 0x44504657U; // "WFPD"
constexpr const char* k_storage_file = "droneos-params.bin";
constexpr const char* k_storage_temp_file = "droneos-params.tmp";

struct StorageParameterEntry {
    std::uint8_t id {0};
    std::uint8_t type {0};
    std::uint16_t reserved {0};
    ParameterValue value {};
};

struct StorageHeader {
    std::uint32_t magic {k_storage_magic};
    std::uint16_t schema_version {ParameterRegistry::k_schema_version};
    std::uint16_t parameter_count {static_cast<std::uint16_t>(ParameterId::count)};
    std::uint16_t header_size {sizeof(StorageHeader)};
    std::uint16_t entry_size {sizeof(StorageParameterEntry)};
    std::uint32_t checksum {0};
};

std::uint32_t checksum_parameters(const StorageParameterEntry* entries, std::size_t count)
{
    const auto* bytes = reinterpret_cast<const std::uint8_t*>(entries);
    const std::size_t length = sizeof(StorageParameterEntry) * count;

    std::uint32_t crc = 0x811C9DC5U;
    for (std::size_t index = 0; index < length; ++index) {
        crc ^= bytes[index];
        crc *= 16777619U;
    }

    return crc;
}

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
    dirty_ = true;
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
    dirty_ = true;
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
    dirty_ = true;
    return true;
}

void ParameterRegistry::reset_to_defaults()
{
    for (std::size_t i = 0; i < parameter_count_; ++i) {
        parameters_[i].value = parameters_[i].default_value;
    }

    dirty_ = true;
}

bool ParameterRegistry::load_from_storage()
{
    FILE* file = std::fopen(k_storage_file, "rb");
    if (file == nullptr) {
        return false;
    }

    StorageHeader header {};
    const std::size_t header_read = std::fread(&header, sizeof(header), 1U, file);
    if (header_read != 1U ||
        header.magic != k_storage_magic ||
        header.schema_version != k_schema_version ||
        header.parameter_count != static_cast<std::uint16_t>(parameter_count_) ||
        header.header_size != sizeof(StorageHeader) ||
        header.entry_size != sizeof(StorageParameterEntry)) {
        std::fclose(file);
        return false;
    }

    StorageParameterEntry stored_entries[parameter_count_] {};
    const std::size_t params_read =
        std::fread(stored_entries, sizeof(StorageParameterEntry), parameter_count_, file);
    std::fclose(file);
    if (params_read != parameter_count_) {
        return false;
    }

    const std::uint32_t observed_checksum =
        checksum_parameters(stored_entries, parameter_count_);
    if (observed_checksum != header.checksum) {
        return false;
    }

    ParameterValue loaded_values[parameter_count_] {};
    for (std::size_t index = 0; index < parameter_count_; ++index) {
        if (stored_entries[index].id != static_cast<std::uint8_t>(parameters_[index].id) ||
            stored_entries[index].type != static_cast<std::uint8_t>(parameters_[index].type)) {
            return false;
        }

        if (!value_in_range(parameters_[index], stored_entries[index].value)) {
            return false;
        }

        loaded_values[index] = stored_entries[index].value;
    }

    for (std::size_t index = 0; index < parameter_count_; ++index) {
        parameters_[index].value = loaded_values[index];
    }

    dirty_ = false;
    return true;
}

bool ParameterRegistry::save_to_storage()
{
    if (!dirty_) {
        return true;
    }

    StorageParameterEntry entries[parameter_count_] {};
    for (std::size_t index = 0; index < parameter_count_; ++index) {
        entries[index].id = static_cast<std::uint8_t>(parameters_[index].id);
        entries[index].type = static_cast<std::uint8_t>(parameters_[index].type);
        entries[index].value = parameters_[index].value;
    }

    StorageHeader header {};
    header.checksum = checksum_parameters(entries, parameter_count_);

    FILE* file = std::fopen(k_storage_temp_file, "wb");
    if (file == nullptr) {
        return false;
    }

    const std::size_t header_written = std::fwrite(&header, sizeof(header), 1U, file);
    const std::size_t params_written =
        std::fwrite(entries, sizeof(StorageParameterEntry), parameter_count_, file);
    const int flush_result = std::fflush(file);
    const int close_result = std::fclose(file);

    if (header_written != 1U ||
        params_written != parameter_count_ ||
        flush_result != 0 ||
        close_result != 0) {
        (void) std::remove(k_storage_temp_file);
        return false;
    }

    (void) std::remove(k_storage_file);
    if (std::rename(k_storage_temp_file, k_storage_file) != 0) {
        (void) std::remove(k_storage_temp_file);
        return false;
    }

    dirty_ = false;
    return true;
}

std::uint16_t ParameterRegistry::schema_version() const
{
    return k_schema_version;
}

bool ParameterRegistry::dirty() const
{
    return dirty_;
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

bool ParameterRegistry::value_in_range(const Parameter& parameter, ParameterValue value)
{
    switch (parameter.type) {
    case ParameterType::float32:
        return value.float32 >= parameter.min_value.float32 &&
               value.float32 <= parameter.max_value.float32;
    case ParameterType::int32:
        return value.int32 >= parameter.min_value.int32 &&
               value.int32 <= parameter.max_value.int32;
    case ParameterType::boolean:
        return value.boolean == parameter.min_value.boolean ||
               value.boolean == parameter.max_value.boolean;
    }

    return false;
}

} // namespace dfw::config
