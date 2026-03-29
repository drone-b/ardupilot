#include "future/HardwareGraph.hpp"

namespace dfw::future {

namespace {

std::uint32_t crc32_update(std::uint32_t seed, const std::uint8_t* data, std::size_t length)
{
    std::uint32_t crc = seed;

    for (std::size_t index = 0; index < length; ++index) {
        crc ^= static_cast<std::uint32_t>(data[index]);
        for (std::uint8_t bit = 0; bit < 8U; ++bit) {
            const std::uint32_t mask = static_cast<std::uint32_t>(-(crc & 1U));
            crc = (crc >> 1U) ^ (0xEDB88320U & mask);
        }
    }

    return crc;
}

} // namespace

void HardwareGraph::clear()
{
    nodes_ = {};
    edges_ = {};
    node_count_ = 0;
    edge_count_ = 0;
}

bool HardwareGraph::add_node(const NodeDescriptor& node)
{
    if (node_count_ >= k_max_nodes) {
        return false;
    }

    if (!node.present) {
        return false;
    }

    nodes_[node_count_] = node;
    ++node_count_;
    return true;
}

bool HardwareGraph::add_edge(const EdgeDescriptor& edge)
{
    if (edge_count_ >= k_max_edges) {
        return false;
    }

    if (!edge.present) {
        return false;
    }

    if (edge.source_node >= node_count_ || edge.target_node >= node_count_) {
        return false;
    }

    edges_[edge_count_] = edge;
    ++edge_count_;
    return true;
}

GraphSnapshot HardwareGraph::snapshot() const
{
    GraphSnapshot out {};
    out.nodes = nodes_;
    out.edges = edges_;
    out.node_count = node_count_;
    out.edge_count = edge_count_;
    out.fingerprint_crc32 = compute_fingerprint(nodes_, node_count_);
    return out;
}

std::uint32_t HardwareGraph::compute_fingerprint(
    const std::array<NodeDescriptor, k_max_nodes>& nodes,
    std::size_t node_count)
{
    std::array<std::uint64_t, k_max_nodes> ordered_tokens {};

    for (std::size_t index = 0; index < node_count; ++index) {
        ordered_tokens[index] =
            (static_cast<std::uint64_t>(nodes[index].component_id) << 32U) |
            static_cast<std::uint64_t>(nodes[index].serial_hash);
    }

    for (std::size_t i = 1; i < node_count; ++i) {
        const std::uint64_t value = ordered_tokens[i];
        std::size_t j = i;
        while (j > 0 && ordered_tokens[j - 1] > value) {
            ordered_tokens[j] = ordered_tokens[j - 1];
            --j;
        }

        ordered_tokens[j] = value;
    }

    std::uint32_t crc = 0xFFFFFFFFU;
    for (std::size_t index = 0; index < node_count; ++index) {
        const std::uint64_t token = ordered_tokens[index];
        std::uint8_t bytes[8] {};
        for (std::uint8_t offset = 0; offset < 8U; ++offset) {
            bytes[offset] = static_cast<std::uint8_t>((token >> (offset * 8U)) & 0xFFU);
        }

        crc = crc32_update(crc, bytes, sizeof(bytes));
    }

    return ~crc;
}

} // namespace dfw::future
