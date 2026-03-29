#pragma once

#include "future/HardwareGraph.hpp"

#include <cstdint>

namespace dfw::future {

struct FactoryTestResult {
    bool supported {false};
    bool passed {false};
    std::uint32_t detail_code {0};
};

class DroneFactoryBridge {
public:
    void set_graph_snapshot(const GraphSnapshot& snapshot);
    const GraphSnapshot& graph_snapshot() const;

    FactoryTestResult run_named_test(const char* test_name) const;
    bool apply_calibration_blob(std::uint32_t schema_version, std::uint32_t blob_crc32) const;

private:
    static bool str_equal(const char* left, const char* right);

    GraphSnapshot snapshot_ {};
    std::uint32_t calibration_schema_version_ {1};
};

} // namespace dfw::future
