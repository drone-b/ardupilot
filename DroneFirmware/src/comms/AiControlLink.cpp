#include "comms/AiControlLink.hpp"

namespace dfw::comms {

bool AiControlLink::ingest_wrench(const AiWrenchCommandV1& command,
                                  common::TimestampUs now_us,
                                  common::DurationUs max_latency_us)
{
    if (command.version != 1) {
        return false;
    }

    if (command.t_cmd_us == 0 || command.t_cmd_us > now_us) {
        return false;
    }

    const common::DurationUs age_us =
        static_cast<common::DurationUs>(now_us - command.t_cmd_us);
    if (age_us > max_latency_us) {
        return false;
    }

    if (has_command_ && command.seq <= last_seq_) {
        return false;
    }

    latest_wrench_ = command;
    last_seq_ = command.seq;
    has_command_ = true;
    return true;
}

bool AiControlLink::has_fresh_command(common::TimestampUs now_us,
                                      common::DurationUs max_latency_us) const
{
    return link_state(now_us, max_latency_us) == AiLinkState::fresh;
}

bool AiControlLink::has_command() const
{
    return has_command_;
}

AiLinkState AiControlLink::link_state(common::TimestampUs now_us,
                                      common::DurationUs max_latency_us) const
{
    if (!has_command_) {
        return AiLinkState::missing;
    }

    if (latest_wrench_.t_cmd_us == 0 || now_us < latest_wrench_.t_cmd_us) {
        return AiLinkState::future_dated;
    }

    const common::DurationUs age_us =
        static_cast<common::DurationUs>(now_us - latest_wrench_.t_cmd_us);
    if (age_us > max_latency_us) {
        return AiLinkState::stale;
    }

    return AiLinkState::fresh;
}

const AiWrenchCommandV1& AiControlLink::latest_wrench() const
{
    return latest_wrench_;
}

} // namespace dfw::comms
