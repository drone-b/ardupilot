#include "future/DroneFactoryBridge.hpp"

#include <cstddef>

namespace dfw::future {

void DroneFactoryBridge::set_graph_snapshot(const GraphSnapshot& snapshot)
{
    snapshot_ = snapshot;
}

const GraphSnapshot& DroneFactoryBridge::graph_snapshot() const
{
    return snapshot_;
}

FactoryTestResult DroneFactoryBridge::run_named_test(const char* test_name) const
{
    if (test_name == nullptr) {
        return {};
    }

    if (str_equal(test_name, "motor_order")) {
        return FactoryTestResult {true, snapshot_.node_count >= 4U, 0U};
    }

    if (str_equal(test_name, "esc_telemetry")) {
        return FactoryTestResult {true, snapshot_.node_count >= 1U, 0U};
    }

    if (str_equal(test_name, "sensor_self_test")) {
        return FactoryTestResult {true, snapshot_.node_count >= 1U, 0U};
    }

    return {};
}

bool DroneFactoryBridge::apply_calibration_blob(std::uint32_t schema_version,
                                                std::uint32_t blob_crc32) const
{
    if (schema_version != calibration_schema_version_) {
        return false;
    }

    return blob_crc32 != 0U;
}

bool DroneFactoryBridge::str_equal(const char* left, const char* right)
{
    if (left == nullptr || right == nullptr) {
        return false;
    }

    std::size_t index = 0;
    while (left[index] != '\0' && right[index] != '\0') {
        if (left[index] != right[index]) {
            return false;
        }

        ++index;
    }

    return left[index] == '\0' && right[index] == '\0';
}

} // namespace dfw::future
