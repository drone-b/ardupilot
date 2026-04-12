#pragma once

#include "platform/Hal.hpp"
#include "sensing/Sensor.hpp"

namespace dfw::platform::stm32 {

class Stm32ImuDevice final : public sensing::IImuDevice {
public:
    explicit Stm32ImuDevice(ISpiDevice& spi_device);

    bool initialize() override;
    bool read_sample(common::TimestampUs trigger_time_us,
                     common::TimestampUs acquisition_time_us,
                     sensing::ImuSample& out_sample) override;

private:
    ISpiDevice& spi_device_;
    bool initialized_ {false};
};

class Stm32BarometerDevice final : public sensing::IBarometerDevice {
public:
    explicit Stm32BarometerDevice(II2cDevice& i2c_device);

    bool initialize() override;
    bool read_sample(common::TimestampUs trigger_time_us,
                     common::TimestampUs acquisition_time_us,
                     sensing::BarometerSample& out_sample) override;

private:
    II2cDevice& i2c_device_;
    bool initialized_ {false};
};

class Stm32MagnetometerDevice final : public sensing::IMagnetometerDevice {
public:
    explicit Stm32MagnetometerDevice(II2cDevice& i2c_device);

    bool initialize() override;
    bool read_sample(common::TimestampUs trigger_time_us,
                     common::TimestampUs acquisition_time_us,
                     sensing::MagnetometerSample& out_sample) override;

private:
    II2cDevice& i2c_device_;
    bool initialized_ {false};
};

} // namespace dfw::platform::stm32
