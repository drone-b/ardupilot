#pragma once

#include "common/TimeTypes.hpp"
#include "platform/Hal.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace dfw::runtime {

class Scheduler {
public:
    using TaskCallback = void (*)(void*, common::TimestampUs);

    enum class PriorityClass : std::uint8_t {
        estimation_fast = 0,
        critical_fast,
        mission_normal,
        service_background
    };

    enum class ReleaseMode : std::uint8_t {
        periodic,
        event_driven
    };

    struct TaskTiming {
        common::DurationUs last_execution_us {0};
        common::DurationUs max_execution_us {0};
        std::uint32_t overrun_count {0};
        std::uint32_t skipped_release_count {0};
        std::uint32_t slack_denial_count {0};
    };

    struct SchedulerStats {
        std::uint32_t skipped_release_count {0};
        std::uint32_t slack_denial_count {0};
        std::uint32_t mode_transition_count {0};
    };

    enum class SchedulerMode : std::uint8_t {
        normal = 0,
        degraded,
        emergency
    };

    struct TaskSpec {
        const char* name {nullptr};
        PriorityClass priority {PriorityClass::mission_normal};
        ReleaseMode release_mode {ReleaseMode::periodic};
        common::DurationUs period_us {0};
        common::DurationUs budget_us {0};
        void* context {nullptr};
        TaskCallback callback {};
        common::TimestampUs next_release_us {0};
        bool pending_event {false};
        std::uint32_t deadline_misses {0};
        TaskTiming timing {};
    };

    static constexpr std::size_t k_max_tasks = 16;

    explicit Scheduler(platform::IClock& clock);

    bool add_task(const char* name,
                  PriorityClass priority,
                  common::DurationUs period_us,
                  common::DurationUs budget_us,
                  void* context,
                  TaskCallback callback);
    bool add_event_task(const char* name,
                        PriorityClass priority,
                        common::DurationUs budget_us,
                        void* context,
                        TaskCallback callback);
    void notify_event(const char* name);
    void run_once();
    std::size_t task_count() const;
    const TaskSpec* task_at(std::size_t index) const;
    const SchedulerStats& stats() const;
    SchedulerMode mode() const;

private:
    static constexpr std::uint32_t k_degraded_entry_violations = 3;
    static constexpr std::uint32_t k_emergency_entry_violations = 10;
    static constexpr std::uint32_t k_recovery_clean_cycles = 200;

    common::TimestampUs next_higher_priority_release(PriorityClass priority) const;
    void skip_periodic_release(TaskSpec& task, common::TimestampUs now_us);
    void run_priority_class(PriorityClass priority, common::TimestampUs now_us);
    void update_mode_end_of_cycle();

    platform::IClock& clock_;
    std::array<TaskSpec, k_max_tasks> tasks_ {};
    std::size_t task_count_ {0};
    SchedulerStats stats_ {};
    SchedulerMode mode_ {SchedulerMode::normal};
    bool cycle_fast_violation_ {false};
    std::uint32_t consecutive_fast_violations_ {0};
    std::uint32_t consecutive_clean_cycles_ {0};
};

} // namespace dfw::runtime
