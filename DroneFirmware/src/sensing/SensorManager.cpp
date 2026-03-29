#include "sensing/SensorManager.hpp"

namespace dfw::sensing {

SensorManager::SensorManager(IImuDevice& imu_device) :
    imu_device_(imu_device)
{
}

bool SensorManager::initialize()
{
    return imu_device_.initialize();
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

    const std::uint8_t published_index =
        imu_buffer_.published_index.load(std::memory_order_relaxed);
    const std::uint8_t write_index = published_index == 0 ? 1 : 0;

    ImuSample sample {};
    if (!imu_device_.read_sample(trigger_time_us, acquisition_time_us, sample)) {
        return false;
    }

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

} // namespace dfw::sensing
