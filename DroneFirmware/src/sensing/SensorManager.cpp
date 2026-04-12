#include "sensing/SensorManager.hpp"

namespace dfw::sensing {

SensorManager::SensorManager(IImuDevice& imu_device) :
    imu_device_(imu_device)
{
}

SensorManager::SensorManager(IImuDevice& imu_device,
                             IBarometerDevice* barometer_device,
                             IMagnetometerDevice* magnetometer_device) :
    imu_device_(imu_device),
    barometer_device_(barometer_device),
    magnetometer_device_(magnetometer_device)
{
}

bool SensorManager::initialize()
{
    const bool imu_initialized = imu_device_.initialize();
    imu_health_.initialized = imu_initialized;
    imu_health_.healthy = imu_initialized;
    imu_health_.last_status = SensorHealthStatus::no_data;

    bool optional_initialized = true;
    if (barometer_device_ != nullptr) {
        const bool initialized = barometer_device_->initialize();
        barometer_health_.initialized = initialized;
        barometer_health_.healthy = initialized;
        barometer_health_.last_status = SensorHealthStatus::no_data;
        optional_initialized = optional_initialized && initialized;
    }

    if (magnetometer_device_ != nullptr) {
        const bool initialized = magnetometer_device_->initialize();
        magnetometer_health_.initialized = initialized;
        magnetometer_health_.healthy = initialized;
        magnetometer_health_.last_status = SensorHealthStatus::no_data;
        optional_initialized = optional_initialized && initialized;
    }

    return imu_initialized && optional_initialized;
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
        mark_sensor_failure(imu_health_,
                            acquisition_time_us,
                            SensorHealthStatus::invalid_timestamp);
        return false;
    }

    const std::uint8_t published_index =
        imu_buffer_.published_index.load(std::memory_order_relaxed);
    const std::uint8_t write_index = published_index == 0 ? 1 : 0;

    ImuSample sample {};
    if (!imu_device_.read_sample(trigger_time_us, acquisition_time_us, sample)) {
        mark_sensor_failure(imu_health_, acquisition_time_us, SensorHealthStatus::no_data);
        return false;
    }
    apply_imu_calibration(sample);

    if (last_published_sample_time_us_ != 0 && trigger_time_us >= last_published_sample_time_us_) {
        sample.sample_period_us =
            static_cast<common::DurationUs>(trigger_time_us - last_published_sample_time_us_);
    }

    sample.sequence = next_imu_sequence_++;
    sample.valid = true;
    sample.status = ImuSampleStatus::ok;

    imu_buffer_.slots[write_index] = sample;
    imu_buffer_.published_index.store(write_index, std::memory_order_release);
    imu_buffer_.has_sample.store(true, std::memory_order_release);
    last_published_sample_time_us_ = trigger_time_us;
    mark_sensor_success(imu_health_, acquisition_time_us);
    return true;
}

bool SensorManager::acquire_barometer(common::TimestampUs trigger_time_us,
                                      common::TimestampUs acquisition_time_us)
{
    if (barometer_device_ == nullptr || !barometer_health_.initialized) {
        return false;
    }

    if (trigger_time_us == 0U || acquisition_time_us < trigger_time_us) {
        mark_sensor_failure(barometer_health_,
                            acquisition_time_us,
                            SensorHealthStatus::invalid_timestamp);
        return false;
    }

    const std::uint8_t published_index =
        barometer_buffer_.published_index.load(std::memory_order_relaxed);
    const std::uint8_t write_index = published_index == 0 ? 1 : 0;

    BarometerSample sample {};
    if (!barometer_device_->read_sample(trigger_time_us, acquisition_time_us, sample)) {
        mark_sensor_failure(barometer_health_, acquisition_time_us, SensorHealthStatus::no_data);
        return false;
    }

    if (!sample.valid || sample.status != BarometerSampleStatus::ok) {
        mark_sensor_failure(barometer_health_,
                            acquisition_time_us,
                            to_health_status(sample.status));
        return false;
    }

    sample.sequence = next_barometer_sequence_++;
    barometer_buffer_.slots[write_index] = sample;
    barometer_buffer_.published_index.store(write_index, std::memory_order_release);
    barometer_buffer_.has_sample.store(true, std::memory_order_release);
    mark_sensor_success(barometer_health_, acquisition_time_us);
    return true;
}

bool SensorManager::acquire_magnetometer(common::TimestampUs trigger_time_us,
                                         common::TimestampUs acquisition_time_us)
{
    if (magnetometer_device_ == nullptr || !magnetometer_health_.initialized) {
        return false;
    }

    if (trigger_time_us == 0U || acquisition_time_us < trigger_time_us) {
        mark_sensor_failure(magnetometer_health_,
                            acquisition_time_us,
                            SensorHealthStatus::invalid_timestamp);
        return false;
    }

    const std::uint8_t published_index =
        magnetometer_buffer_.published_index.load(std::memory_order_relaxed);
    const std::uint8_t write_index = published_index == 0 ? 1 : 0;

    MagnetometerSample sample {};
    if (!magnetometer_device_->read_sample(trigger_time_us, acquisition_time_us, sample)) {
        mark_sensor_failure(magnetometer_health_,
                            acquisition_time_us,
                            SensorHealthStatus::no_data);
        return false;
    }

    if (!sample.valid || sample.status != MagnetometerSampleStatus::ok) {
        mark_sensor_failure(magnetometer_health_,
                            acquisition_time_us,
                            to_health_status(sample.status));
        return false;
    }

    sample.sequence = next_magnetometer_sequence_++;
    magnetometer_buffer_.slots[write_index] = sample;
    magnetometer_buffer_.published_index.store(write_index, std::memory_order_release);
    magnetometer_buffer_.has_sample.store(true, std::memory_order_release);
    mark_sensor_success(magnetometer_health_, acquisition_time_us);
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

bool SensorManager::get_latest_barometer_sample(BarometerSample& out_sample) const
{
    if (!barometer_buffer_.has_sample.load(std::memory_order_acquire)) {
        return false;
    }

    const std::uint8_t read_index =
        barometer_buffer_.published_index.load(std::memory_order_acquire);
    out_sample = barometer_buffer_.slots[read_index];
    return out_sample.valid;
}

bool SensorManager::get_latest_magnetometer_sample(MagnetometerSample& out_sample) const
{
    if (!magnetometer_buffer_.has_sample.load(std::memory_order_acquire)) {
        return false;
    }

    const std::uint8_t read_index =
        magnetometer_buffer_.published_index.load(std::memory_order_acquire);
    out_sample = magnetometer_buffer_.slots[read_index];
    return out_sample.valid;
}

const SensorHealth& SensorManager::imu_health() const
{
    return imu_health_;
}

const SensorHealth& SensorManager::barometer_health() const
{
    return barometer_health_;
}

const SensorHealth& SensorManager::magnetometer_health() const
{
    return magnetometer_health_;
}

void SensorManager::set_imu_calibration(const ImuCalibration& calibration)
{
    imu_calibration_ = calibration;
}

void SensorManager::mark_sensor_success(SensorHealth& health, common::TimestampUs now_us)
{
    health.last_update_us = now_us;
    ++health.total_sample_count;
    health.consecutive_error_count = 0;
    health.healthy = health.initialized;
    health.last_status = SensorHealthStatus::ok;
}

void SensorManager::mark_sensor_failure(SensorHealth& health,
                                        common::TimestampUs now_us,
                                        SensorHealthStatus status)
{
    health.last_update_us = now_us;
    ++health.total_error_count;
    ++health.consecutive_error_count;
    health.healthy = false;
    health.last_status = status;
}

SensorHealthStatus SensorManager::to_health_status(BarometerSampleStatus status)
{
    switch (status) {
    case BarometerSampleStatus::ok:
        return SensorHealthStatus::ok;
    case BarometerSampleStatus::invalid_timestamp:
        return SensorHealthStatus::invalid_timestamp;
    case BarometerSampleStatus::no_data:
        return SensorHealthStatus::no_data;
    case BarometerSampleStatus::invalid_pressure:
    case BarometerSampleStatus::invalid_temperature:
        return SensorHealthStatus::invalid_sample;
    }

    return SensorHealthStatus::invalid_sample;
}

SensorHealthStatus SensorManager::to_health_status(MagnetometerSampleStatus status)
{
    switch (status) {
    case MagnetometerSampleStatus::ok:
        return SensorHealthStatus::ok;
    case MagnetometerSampleStatus::invalid_timestamp:
        return SensorHealthStatus::invalid_timestamp;
    case MagnetometerSampleStatus::no_data:
        return SensorHealthStatus::no_data;
    case MagnetometerSampleStatus::invalid_field:
    case MagnetometerSampleStatus::invalid_temperature:
        return SensorHealthStatus::invalid_sample;
    }

    return SensorHealthStatus::invalid_sample;
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
