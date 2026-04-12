#pragma once

#include "common/TimeTypes.hpp"
#include "sensing/Sensor.hpp"

#include <atomic>

namespace dfw::sensing {

class SensorManager {
public:
    explicit SensorManager(IImuDevice& imu_device);

    bool initialize();

    // Called from the IMU data-ready callback. This does not read the bus.
    void notify_imu_trigger(common::TimestampUs trigger_time_us);

    // Called from the scheduled imu_acquisition task.
    bool acquire_imu(common::TimestampUs acquisition_time_us);

    // Called from periodic control or estimation tasks.
    bool get_latest_imu_sample(ImuSample& out_sample) const;
    const SensorHealth& imu_health() const;
    void set_imu_calibration(const ImuCalibration& calibration);

private:
    struct ImuBuffer {
        ImuSample slots[2] {};
        std::atomic<std::uint8_t> published_index {0};
        std::atomic<bool> has_sample {false};
    };

    IImuDevice& imu_device_;
    ImuBuffer imu_buffer_ {};
    ImuCalibration imu_calibration_ {};
    SensorHealth imu_health_ {};
    std::atomic<bool> imu_trigger_pending_ {false};
    std::atomic<common::TimestampUs> imu_trigger_time_us_ {0};
    std::uint32_t next_sequence_ {1};
    common::TimestampUs last_published_sample_time_us_ {0};

    void mark_imu_success(common::TimestampUs now_us);
    void mark_imu_failure(common::TimestampUs now_us, ImuSampleStatus status);
    void apply_imu_calibration(ImuSample& sample) const;
};

} // namespace dfw::sensing
