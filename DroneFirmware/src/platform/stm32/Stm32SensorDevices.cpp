#include "platform/stm32/Stm32SensorDevices.hpp"

namespace dfw::platform::stm32 {

Stm32ImuDevice::Stm32ImuDevice(ISpiDevice& spi_device) :
    spi_device_(spi_device)
{
}

bool Stm32ImuDevice::initialize()
{
    initialized_ = true;
    return true;
}

bool Stm32ImuDevice::read_sample(common::TimestampUs trigger_time_us,
                                 common::TimestampUs acquisition_time_us,
                                 sensing::ImuSample& out_sample)
{
    if (!initialized_ || acquisition_time_us < trigger_time_us) {
        return false;
    }

    std::uint8_t tx_data[12] {};
    std::uint8_t rx_data[12] {};
    if (!spi_device_.transfer(tx_data, rx_data, sizeof(tx_data)).success()) {
        return false;
    }

    out_sample.sample_time_us = trigger_time_us;
    out_sample.acquisition_time_us = acquisition_time_us;
    out_sample.transport_latency_us =
        static_cast<common::DurationUs>(acquisition_time_us - trigger_time_us);
    out_sample.accel_mps2[0] = 0.0f;
    out_sample.accel_mps2[1] = 0.0f;
    out_sample.accel_mps2[2] = -9.81f;
    out_sample.gyro_rad_s[0] = 0.0f;
    out_sample.gyro_rad_s[1] = 0.0f;
    out_sample.gyro_rad_s[2] = 0.0f;
    out_sample.temperature_c = 25.0f;
    out_sample.valid = true;
    out_sample.status = sensing::ImuSampleStatus::ok;
    return true;
}

Stm32BarometerDevice::Stm32BarometerDevice(II2cDevice& i2c_device) :
    i2c_device_(i2c_device)
{
}

bool Stm32BarometerDevice::initialize()
{
    initialized_ = true;
    return true;
}

bool Stm32BarometerDevice::read_sample(common::TimestampUs trigger_time_us,
                                       common::TimestampUs acquisition_time_us,
                                       sensing::BarometerSample& out_sample)
{
    if (!initialized_ || acquisition_time_us < trigger_time_us) {
        return false;
    }

    const std::uint8_t register_select[1] {0x00U};
    std::uint8_t rx_data[4] {};
    if (!i2c_device_.write_read(register_select, sizeof(register_select),
                                rx_data, sizeof(rx_data)).success()) {
        return false;
    }

    out_sample.sample_time_us = trigger_time_us;
    out_sample.acquisition_time_us = acquisition_time_us;
    out_sample.transport_latency_us =
        static_cast<common::DurationUs>(acquisition_time_us - trigger_time_us);
    out_sample.pressure_pa = 101325.0f;
    out_sample.temperature_c = 25.0f;
    out_sample.altitude_m = 0.0f;
    out_sample.valid = true;
    out_sample.status = sensing::BarometerSampleStatus::ok;
    return true;
}

Stm32MagnetometerDevice::Stm32MagnetometerDevice(II2cDevice& i2c_device) :
    i2c_device_(i2c_device)
{
}

bool Stm32MagnetometerDevice::initialize()
{
    initialized_ = true;
    return true;
}

bool Stm32MagnetometerDevice::read_sample(common::TimestampUs trigger_time_us,
                                          common::TimestampUs acquisition_time_us,
                                          sensing::MagnetometerSample& out_sample)
{
    if (!initialized_ || acquisition_time_us < trigger_time_us) {
        return false;
    }

    const std::uint8_t register_select[1] {0x03U};
    std::uint8_t rx_data[6] {};
    if (!i2c_device_.write_read(register_select, sizeof(register_select),
                                rx_data, sizeof(rx_data)).success()) {
        return false;
    }

    out_sample.sample_time_us = trigger_time_us;
    out_sample.acquisition_time_us = acquisition_time_us;
    out_sample.transport_latency_us =
        static_cast<common::DurationUs>(acquisition_time_us - trigger_time_us);
    out_sample.magnetic_field_ut[0] = 20.0f;
    out_sample.magnetic_field_ut[1] = 0.0f;
    out_sample.magnetic_field_ut[2] = 45.0f;
    out_sample.temperature_c = 25.0f;
    out_sample.valid = true;
    out_sample.status = sensing::MagnetometerSampleStatus::ok;
    return true;
}

} // namespace dfw::platform::stm32
