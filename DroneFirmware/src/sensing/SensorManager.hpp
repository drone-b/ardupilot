#pragma once

#include "common/TimeTypes.hpp"
#include "sensing/Sensor.hpp"

#include <atomic>

namespace dfw::sensing {

class SensorManager {
public:
    explicit SensorManager(IImuDevice& imu_device);
    SensorManager(IImuDevice& imu_device,
                  IBarometerDevice* barometer_device,
                  IMagnetometerDevice* magnetometer_device);

    bool initialize();

    // Called from the IMU data-ready callback. This does not read the bus.
    void notify_imu_trigger(common::TimestampUs trigger_time_us);

    // Called from the scheduled imu_acquisition task.
    bool acquire_imu(common::TimestampUs acquisition_time_us);
    bool acquire_barometer(common::TimestampUs trigger_time_us,
                           common::TimestampUs acquisition_time_us);
    bool acquire_magnetometer(common::TimestampUs trigger_time_us,
                              common::TimestampUs acquisition_time_us);

    // Called from periodic control or estimation tasks.
    bool get_latest_imu_sample(ImuSample& out_sample) const;
    bool get_latest_barometer_sample(BarometerSample& out_sample) const;
    bool get_latest_magnetometer_sample(MagnetometerSample& out_sample) const;
    const SensorHealth& imu_health() const;
    const SensorHealth& barometer_health() const;
    const SensorHealth& magnetometer_health() const;
    void set_imu_calibration(const ImuCalibration& calibration);

private:
    template <typename Sample>
    struct SampleBuffer {
        Sample slots[2] {};
        std::atomic<std::uint8_t> published_index {0};
        std::atomic<bool> has_sample {false};
    };

    IImuDevice& imu_device_;
    IBarometerDevice* barometer_device_ {nullptr};
    IMagnetometerDevice* magnetometer_device_ {nullptr};
    SampleBuffer<ImuSample> imu_buffer_ {};
    SampleBuffer<BarometerSample> barometer_buffer_ {};
    SampleBuffer<MagnetometerSample> magnetometer_buffer_ {};
    ImuCalibration imu_calibration_ {};
    SensorHealth imu_health_ {};
    SensorHealth barometer_health_ {};
    SensorHealth magnetometer_health_ {};
    std::atomic<bool> imu_trigger_pending_ {false};
    std::atomic<common::TimestampUs> imu_trigger_time_us_ {0};
    std::uint32_t next_imu_sequence_ {1};
    std::uint32_t next_barometer_sequence_ {1};
    std::uint32_t next_magnetometer_sequence_ {1};
    common::TimestampUs last_published_sample_time_us_ {0};

    static void mark_sensor_success(SensorHealth& health, common::TimestampUs now_us);
    static void mark_sensor_failure(SensorHealth& health,
                                    common::TimestampUs now_us,
                                    SensorHealthStatus status);
    static SensorHealthStatus to_health_status(BarometerSampleStatus status);
    static SensorHealthStatus to_health_status(MagnetometerSampleStatus status);
    void apply_imu_calibration(ImuSample& sample) const;
};

} // namespace dfw::sensing
