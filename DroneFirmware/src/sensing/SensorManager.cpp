#include "sensing/SensorManager.hpp"

namespace dfw::sensing {

SensorManager::SensorManager(IImuDevice& imu_device) :
    imu_device_(imu_device)
{
}

bool SensorManager::initialize()
{
    const bool initialized = imu_device_.initialize();
    imu_health_.initialized = initialized;
    imu_health_.healthy = initialized;
    imu_health_.last_status = initialized ? ImuSampleStatus::no_data : ImuSampleStatus::no_data;
    return initialized;
}

void SensorManager::notify_imu_trigger(common::TimestampUs trigger_time_us)
{
    imu_trigger_time_us_.store(trigger_time_us, std::memory_order_relaxed);
    imu_trigger_pending_.store(true, std::memory_order_release);
}

bool SensorManager::acquire_imu(common::TimestampUs acquisition_time_us)
{
    if (!imu_trigger_pending_.exchange(false, std::memory_order_acq_rel)) {
        return false;
    }

    const common::TimestampUs trigger_time_us =
        imu_trigger_time_us_.load(std::memory_order_acquire);
    if (trigger_time_us == 0U || acquisition_time_us < trigger_time_us) {
        mark_imu_failure(acquisition_time_us, ImuSampleStatus::invalid_timestamp);
        return false;
    }

    const std::uint8_t published_index =
        imu_buffer_.published_index.load(std::memory_order_relaxed);
    const std::uint8_t write_index = published_index == 0 ? 1 : 0;

    ImuSample sample {};
    if (!imu_device_.read_sample(trigger_time_us, acquisition_time_us, sample)) {
        mark_imu_failure(acquisition_time_us, ImuSampleStatus::no_data);
        return false;
    }
    apply_imu_calibration(sample);

    if (last_published_sample_time_us_ != 0 && trigger_time_us >= last_published_sample_time_us_) {
        sample.sample_period_us =
            static_cast<common::DurationUs>(trigger_time_us - last_published_sample_time_us_);
    }

    sample.sequence = next_sequence_++;
    sample.valid = true;
    sample.status = ImuSampleStatus::ok;

    imu_buffer_.slots[write_index] = sample;
    imu_buffer_.published_index.store(write_index, std::memory_order_release);
    imu_buffer_.has_sample.store(true, std::memory_order_release);
    last_published_sample_time_us_ = trigger_time_us;
    mark_imu_success(acquisition_time_us);
    return true;
}

bool SensorManager::get_latest_imu_sample(ImuSample& out_sample) const
{
    if (!imu_buffer_.has_sample.load(std::memory_order_acquire)) {
        return false;
    }

    const std::uint8_t read_index =
        imu_buffer_.published_index.load(std::memory_order_acquire);
    out_sample = imu_buffer_.slots[read_index];
    return out_sample.valid;
}

const SensorHealth& SensorManager::imu_health() const
{
    return imu_health_;
}

void SensorManager::set_imu_calibration(const ImuCalibration& calibration)
{
    imu_calibration_ = calibration;
}

void SensorManager::mark_imu_success(common::TimestampUs now_us)
{
    imu_health_.last_update_us = now_us;
    ++imu_health_.total_sample_count;
    imu_health_.consecutive_error_count = 0;
    imu_health_.healthy = imu_health_.initialized;
    imu_health_.last_status = ImuSampleStatus::ok;
}

void SensorManager::mark_imu_failure(common::TimestampUs now_us, ImuSampleStatus status)
{
    imu_health_.last_update_us = now_us;
    ++imu_health_.total_error_count;
    ++imu_health_.consecutive_error_count;
    imu_health_.healthy = false;
    imu_health_.last_status = status;
}

void SensorManager::apply_imu_calibration(ImuSample& sample) const
{
    if (!imu_calibration_.valid) {
        return;
    }

    for (std::uint8_t axis = 0; axis < 3U; ++axis) {
        sample.accel_mps2[axis] =
            (sample.accel_mps2[axis] - imu_calibration_.accel_bias_mps2[axis]) *
            imu_calibration_.accel_scale[axis];
        sample.gyro_rad_s[axis] =
            (sample.gyro_rad_s[axis] - imu_calibration_.gyro_bias_rad_s[axis]) *
            imu_calibration_.gyro_scale[axis];
    }
}

} // namespace dfw::sensing
