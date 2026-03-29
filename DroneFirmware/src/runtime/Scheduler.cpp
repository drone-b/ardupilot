#include "runtime/Scheduler.hpp"

#include <cstring>

namespace dfw::runtime {

Scheduler::Scheduler(platform::IClock& clock) :
    clock_(clock)
{
}

bool Scheduler::add_task(const char* name,
                         PriorityClass priority,
                         common::DurationUs period_us,
                         common::DurationUs budget_us,
                         void* context,
                         TaskCallback callback)
{
    if (task_count_ >= k_max_tasks || callback == nullptr) {
        return false;
    }

    const common::TimestampUs now_us = clock_.now_us();

    tasks_[task_count_++] = TaskSpec {
        name,
        priority,
        ReleaseMode::periodic,
        period_us,
        budget_us,
        context,
        callback,
        now_us + period_us,
        false,
        0,
        {},
    };

    return true;
}

bool Scheduler::add_event_task(const char* name,
                               PriorityClass priority,
                               common::DurationUs budget_us,
                               void* context,
                               TaskCallback callback)
{
    if (task_count_ >= k_max_tasks || callback == nullptr) {
        return false;
    }

    tasks_[task_count_++] = TaskSpec {
        name,
        priority,
        ReleaseMode::event_driven,
        0,
        budget_us,
        context,
        callback,
        0,
        false,
        0,
        {},
    };

    return true;
}

void Scheduler::notify_event(const char* name)
{
    for (std::size_t index = 0; index < task_count_; ++index) {
        TaskSpec& task = tasks_[index];
        if (task.name == nullptr || name == nullptr) {
            continue;
        }

        if (task.release_mode == ReleaseMode::event_driven &&
            std::strcmp(task.name, name) == 0) {
            task.pending_event = true;
        }
    }
}

void Scheduler::run_once()
{
    cycle_fast_violation_ = false;

    run_priority_class(PriorityClass::estimation_fast, clock_.now_us());
    run_priority_class(PriorityClass::critical_fast, clock_.now_us());

    if (mode_ != SchedulerMode::emergency) {
        run_priority_class(PriorityClass::mission_normal, clock_.now_us());
    }

    if (mode_ == SchedulerMode::normal) {
        run_priority_class(PriorityClass::service_background, clock_.now_us());
    }

    update_mode_end_of_cycle();
}

void Scheduler::run_priority_class(PriorityClass priority, common::TimestampUs now_us)
{
    for (std::size_t index = 0; index < task_count_; ++index) {
        TaskSpec& task = tasks_[index];
        if (task.priority != priority || !task.callback) {
            continue;
        }

        const bool due_periodic =
            task.release_mode == ReleaseMode::periodic && now_us >= task.next_release_us;
        const bool due_event =
            task.release_mode == ReleaseMode::event_driven && task.pending_event;

        if (!due_periodic && !due_event) {
            continue;
        }

        const common::TimestampUs start_us = clock_.now_us();
        if (task.release_mode == ReleaseMode::periodic &&
            task.budget_us != 0 &&
            start_us > task.next_release_us + task.budget_us) {
            ++task.deadline_misses;
            ++task.timing.skipped_release_count;
            ++stats_.skipped_release_count;
            if (priority == PriorityClass::estimation_fast ||
                priority == PriorityClass::critical_fast) {
                cycle_fast_violation_ = true;
            }
            skip_periodic_release(task, start_us);
            continue;
        }

        if (priority != PriorityClass::critical_fast) {
            const common::TimestampUs next_release_us = next_higher_priority_release(priority);
            if (next_release_us != 0 &&
                task.budget_us != 0 &&
                start_us < next_release_us &&
                static_cast<common::DurationUs>(next_release_us - start_us) < task.budget_us) {
                ++task.timing.slack_denial_count;
                ++stats_.slack_denial_count;
                continue;
            }
        }

        if (task.release_mode == ReleaseMode::periodic && start_us > task.next_release_us) {
            ++task.deadline_misses;
        }

        task.callback(task.context, start_us);
        const common::TimestampUs end_us = clock_.now_us();
        const common::DurationUs execution_us =
            end_us >= start_us ?
            static_cast<common::DurationUs>(end_us - start_us) :
            task.budget_us + 1;

        task.timing.last_execution_us = execution_us;
        if (execution_us > task.timing.max_execution_us) {
            task.timing.max_execution_us = execution_us;
        }
        if (task.budget_us != 0 && execution_us > task.budget_us) {
            ++task.timing.overrun_count;
            if (priority == PriorityClass::estimation_fast ||
                priority == PriorityClass::critical_fast) {
                cycle_fast_violation_ = true;
            }
        }

        if (task.release_mode == ReleaseMode::periodic) {
            skip_periodic_release(task, start_us);
        } else {
            task.pending_event = false;
        }
    }
}

std::size_t Scheduler::task_count() const
{
    return task_count_;
}

const Scheduler::TaskSpec* Scheduler::task_at(std::size_t index) const
{
    if (index >= task_count_) {
        return nullptr;
    }

    return &tasks_[index];
}

const Scheduler::SchedulerStats& Scheduler::stats() const
{
    return stats_;
}

Scheduler::SchedulerMode Scheduler::mode() const
{
    return mode_;
}

common::TimestampUs Scheduler::next_higher_priority_release(PriorityClass priority) const
{
    common::TimestampUs next_release_us = 0;

    for (std::size_t index = 0; index < task_count_; ++index) {
        const TaskSpec& task = tasks_[index];
        if (task.release_mode != ReleaseMode::periodic ||
            task.priority >= priority ||
            task.period_us == 0) {
            continue;
        }

        if (next_release_us == 0 || task.next_release_us < next_release_us) {
            next_release_us = task.next_release_us;
        }
    }

    return next_release_us;
}

void Scheduler::skip_periodic_release(TaskSpec& task, common::TimestampUs now_us)
{
    if (task.period_us == 0) {
        return;
    }

    do {
        task.next_release_us += task.period_us;
    }
    while (task.next_release_us <= now_us);
}

void Scheduler::update_mode_end_of_cycle()
{
    if (cycle_fast_violation_) {
        ++consecutive_fast_violations_;
        consecutive_clean_cycles_ = 0;
    } else {
        consecutive_fast_violations_ = 0;
        ++consecutive_clean_cycles_;
    }

    SchedulerMode next_mode = mode_;
    if (consecutive_fast_violations_ >= k_emergency_entry_violations) {
        next_mode = SchedulerMode::emergency;
    } else if (consecutive_fast_violations_ >= k_degraded_entry_violations) {
        next_mode = SchedulerMode::degraded;
    } else if (consecutive_clean_cycles_ >= k_recovery_clean_cycles) {
        next_mode = SchedulerMode::normal;
    }

    if (next_mode != mode_) {
        mode_ = next_mode;
        ++stats_.mode_transition_count;
    }
}

} // namespace dfw::runtime
