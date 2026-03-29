#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace dfw::future {

enum class BusKind : std::uint8_t {
    internal_virtual = 0,
    dronecan,
    spi,
    i2c,
};

struct NodeDescriptor {
    std::uint32_t component_id {0};
    std::uint32_t serial_hash {0};
    std::uint16_t vendor_id {0};
    std::uint16_t product_id {0};
    std::uint8_t health {0};
    BusKind bus {BusKind::internal_virtual};
    bool present {false};
};

struct EdgeDescriptor {
    std::uint16_t source_node {0};
    std::uint16_t target_node {0};
    std::uint8_t relation_kind {0};
    bool present {false};
};

struct GraphSnapshot {
    static constexpr std::size_t k_max_nodes = 64;
    static constexpr std::size_t k_max_edges = 128;

    std::array<NodeDescriptor, k_max_nodes> nodes {};
    std::array<EdgeDescriptor, k_max_edges> edges {};
    std::size_t node_count {0};
    std::size_t edge_count {0};
    std::uint32_t fingerprint_crc32 {0};
};

class HardwareGraph {
public:
    static constexpr std::size_t k_max_nodes = GraphSnapshot::k_max_nodes;
    static constexpr std::size_t k_max_edges = GraphSnapshot::k_max_edges;

    void clear();
    bool add_node(const NodeDescriptor& node);
    bool add_edge(const EdgeDescriptor& edge);
    GraphSnapshot snapshot() const;

private:
    static std::uint32_t compute_fingerprint(const std::array<NodeDescriptor, k_max_nodes>& nodes,
                                             std::size_t node_count);

    std::array<NodeDescriptor, k_max_nodes> nodes_ {};
    std::array<EdgeDescriptor, k_max_edges> edges_ {};
    std::size_t node_count_ {0};
    std::size_t edge_count_ {0};
};

} // namespace dfw::future
