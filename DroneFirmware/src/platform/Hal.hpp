#pragma once

#include "common/TimeTypes.hpp"

#include <cstddef>
#include <cstdint>

namespace dfw::platform {

enum class StatusCode : std::uint8_t {
    ok = 0,
    busy,
    timeout,
    invalid_argument,
    io_error,
    unsupported
};

struct Status {
    StatusCode code {StatusCode::ok};

    [[nodiscard]] bool success() const
    {
        return code == StatusCode::ok;
    }
};

enum class SpiMode : std::uint8_t {
    mode0 = 0,
    mode1,
    mode2,
    mode3
};

enum class Parity : std::uint8_t {
    none = 0,
    even,
    odd
};

enum class StopBits : std::uint8_t {
    one = 0,
    two
};

enum class PwmProtocol : std::uint8_t {
    analog_us = 0,
    oneshot,
    dshot
};

struct SpiDeviceConfig {
    std::uint32_t bus_hz {1000000};
    SpiMode mode {SpiMode::mode0};
    bool use_dma {true};
};

struct I2cDeviceConfig {
    std::uint32_t bus_hz {400000};
    std::uint16_t address {0};
};

struct UartConfig {
    std::uint32_t baud_rate {115200};
    Parity parity {Parity::none};
    StopBits stop_bits {StopBits::one};
    bool use_dma {true};
};

struct PwmChannelConfig {
    std::uint32_t update_rate_hz {400};
    std::uint16_t min_pulse_us {1000};
    std::uint16_t max_pulse_us {2000};
    PwmProtocol protocol {PwmProtocol::analog_us};
};

class ISpiDevice {
public:
    virtual ~ISpiDevice() = default;

    virtual Status transfer(const std::uint8_t* tx_data,
                            std::uint8_t* rx_data,
                            std::size_t length) = 0;
};

class II2cDevice {
public:
    virtual ~II2cDevice() = default;

    virtual Status write(const std::uint8_t* data, std::size_t length) = 0;
    virtual Status write_read(const std::uint8_t* tx_data,
                              std::size_t tx_length,
                              std::uint8_t* rx_data,
                              std::size_t rx_length) = 0;
};

class IUartPort {
public:
    virtual ~IUartPort() = default;

    // Enqueue-only transmit request for background-context use such as telemetry.
    // This call is expected to be bounded and non-blocking: it should copy or
    // reference the provided bytes into a driver FIFO or start a DMA-backed
    // transfer request, then return immediately with success/failure status.
    // Returning `busy` indicates that the transmit path cannot currently accept
    // the full request and higher layers should drop or retry on a later tick.
    virtual Status write(const std::uint8_t* data, std::size_t length) = 0;
    virtual std::size_t read(std::uint8_t* data, std::size_t max_length) = 0;
    virtual std::size_t available() const = 0;
};

class IPwmBank {
public:
    virtual ~IPwmBank() = default;

    virtual std::size_t channel_count() const = 0;
    virtual Status configure_channel(std::uint8_t channel,
                                     const PwmChannelConfig& config) = 0;
    virtual Status write_normalized(std::uint8_t channel, float normalized_command) = 0;
    virtual void disarm_all() = 0;
};

class IBusFactory {
public:
    virtual ~IBusFactory() = default;

    virtual ISpiDevice* create_spi_device(std::uint8_t bus_index,
                                          std::uint8_t chip_select_index,
                                          const SpiDeviceConfig& config) = 0;
    virtual II2cDevice* create_i2c_device(std::uint8_t bus_index,
                                          const I2cDeviceConfig& config) = 0;
    virtual IUartPort* create_uart_port(std::uint8_t port_index,
                                        const UartConfig& config) = 0;
};

class IClock {
public:
    virtual ~IClock() = default;

    // Monotonic microsecond counter for scheduled task context.
    virtual common::TimestampUs now_us() const = 0;
    // Interrupt-safe timestamp capture backed by the same hardware timebase.
    virtual common::TimestampUs now_us_isr() const = 0;
    virtual void sleep_us(common::DurationUs duration_us) = 0;
};

class IBoardControl {
public:
    virtual ~IBoardControl() = default;

    virtual void service_watchdog() = 0;
    virtual void reset_system() = 0;
};

class IOutputDriver {
public:
    virtual ~IOutputDriver() = default;

    virtual void write_channel(std::uint8_t channel, float normalized_command) = 0;
};

class ISensorInterruptRouter {
public:
    virtual ~ISensorInterruptRouter() = default;

    virtual void register_data_ready_line(std::uint8_t line_index,
                                          void (*callback)(void*),
                                          void* context) = 0;
};

class IHal {
public:
    virtual ~IHal() = default;

    virtual IClock& clock() = 0;
    virtual IBoardControl& board_control() = 0;
    virtual IOutputDriver& output_driver() = 0;
    virtual IBusFactory& buses() = 0;
    virtual IPwmBank& pwm_bank() = 0;
    virtual ISensorInterruptRouter& sensor_interrupts() = 0;
};

} // namespace dfw::platform
