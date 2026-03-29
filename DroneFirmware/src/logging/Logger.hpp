#pragma once

#include "comms/TelemetryPublisher.hpp"

#include <cstddef>

namespace dfw::logging {

class ILogStorage {
public:
    virtual ~ILogStorage() = default;

    virtual bool append(const comms::TelemetryFrame& frame) = 0;
};

template <std::size_t Capacity>
class LogBuffer {
public:
    bool push(const comms::TelemetryFrame& frame)
    {
        if (Capacity == 0) {
            return false;
        }

        if (count_ == Capacity) {
            // Drop the oldest frame to preserve the newest data.
            tail_ = advance(tail_);
            --count_;
            overflowed_ = true;
        }

        frames_[head_] = frame;
        head_ = advance(head_);
        ++count_;
        return true;
    }

    bool pop(comms::TelemetryFrame& out_frame)
    {
        if (count_ == 0) {
            return false;
        }

        out_frame = frames_[tail_];
        tail_ = advance(tail_);
        --count_;
        return true;
    }

    std::size_t size() const
    {
        return count_;
    }

    bool overflowed() const
    {
        return overflowed_;
    }

private:
    static constexpr std::size_t advance(std::size_t index)
    {
        return (index + 1U) % Capacity;
    }

    comms::TelemetryFrame frames_[Capacity] {};
    std::size_t head_ {0};
    std::size_t tail_ {0};
    std::size_t count_ {0};
    bool overflowed_ {false};
};

template <std::size_t Capacity>
class Logger {
public:
    explicit Logger(ILogStorage& storage) :
        storage_(storage)
    {
    }

    void push(const comms::TelemetryFrame& frame)
    {
        (void) buffer_.push(frame);
    }

    void flush()
    {
        comms::TelemetryFrame frame {};
        if (!buffer_.pop(frame)) {
            return;
        }

        (void) storage_.append(frame);
    }

    std::size_t pending() const
    {
        return buffer_.size();
    }

    bool overflowed() const
    {
        return buffer_.overflowed();
    }

private:
    ILogStorage& storage_;
    LogBuffer<Capacity> buffer_ {};
};

} // namespace dfw::logging
