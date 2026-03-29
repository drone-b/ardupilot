#pragma once

#include "common/TimeTypes.hpp"

#include <cstdint>

namespace dfw::comms {

enum class AiLinkState : std::uint8_t {
    fresh = 0,
    missing = 1,
    stale = 2,
    future_dated = 3,
};

struct AiWrenchCommandV1 {
    std::uint16_t version {1};
    std::uint16_t flags {0};
    std::uint32_t seq {0};
    common::TimestampUs t_cmd_us {0};
    float tau[3] {0.0f, 0.0f, 0.0f};
    float thrust {0.0f};
};

class AiControlLink {
public:
    bool ingest_wrench(const AiWrenchCommandV1& command,
                       common::TimestampUs now_us,
                       common::DurationUs max_latency_us);
    bool has_fresh_command(common::TimestampUs now_us,
                           common::DurationUs max_latency_us) const;
    bool has_command() const;
    AiLinkState link_state(common::TimestampUs now_us,
                           common::DurationUs max_latency_us) const;
    const AiWrenchCommandV1& latest_wrench() const;

private:
    AiWrenchCommandV1 latest_wrench_ {};
    std::uint32_t last_seq_ {0};
    bool has_command_ {false};
};

} // namespace dfw::comms
