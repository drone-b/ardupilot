#pragma once

#include "platform/Hal.hpp"
#include <array>
#include <cstddef>
#include <cstdint>

namespace dfw::platform::stm32 {

class Stm32SpiDevice final : public ISpiDevice {
public:
    struct TransferStats {
        std::uint32_t accepted_transfer_count {0};
        std::uint32_t busy_transfer_count {0};
        std::uint32_t invalid_transfer_count {0};
        std::uint32_t total_transferred_bytes {0};
    };

    Status transfer(const std::uint8_t* tx_data,
                    std::uint8_t* rx_data,
                    std::size_t length) override;
    const TransferStats& transfer_stats() const;
    void set_busy(bool busy);

private:
    static constexpr std::size_t k_max_transfer_bytes = 64;

    bool busy_ {false};
    TransferStats transfer_stats_ {};
};

class Stm32I2cDevice final : public II2cDevice {
public:
    struct TransferStats {
        std::uint32_t accepted_transfer_count {0};
        std::uint32_t busy_transfer_count {0};
        std::uint32_t invalid_transfer_count {0};
        std::uint32_t total_written_bytes {0};
        std::uint32_t total_read_bytes {0};
    };

    Status write(const std::uint8_t* data, std::size_t length) override;
    Status write_read(const std::uint8_t* tx_data,
                      std::size_t tx_length,
                      std::uint8_t* rx_data,
                      std::size_t rx_length) override;
    const TransferStats& transfer_stats() const;
    void set_busy(bool busy);

private:
    static constexpr std::size_t k_max_write_bytes = 32;
    static constexpr std::size_t k_max_read_bytes = 32;

    bool busy_ {false};
    TransferStats transfer_stats_ {};
};

class Stm32UartPort final : public IUartPort {
public:
    struct TxStats {
        std::uint32_t accepted_write_count {0};
        std::uint32_t busy_write_count {0};
        std::uint32_t invalid_write_count {0};
        std::uint32_t dma_start_count {0};
        std::uint32_t dma_complete_count {0};
        std::uint32_t total_enqueued_bytes {0};
        std::uint32_t total_transmitted_bytes {0};
        std::uint16_t max_queued_bytes {0};
    };

    Status write(const std::uint8_t* data, std::size_t length) override;
    std::size_t read(std::uint8_t* data, std::size_t max_length) override;
    std::size_t available() const override;
    std::size_t queued_tx_bytes() const;
    std::size_t dma_inflight_bytes() const;
    bool tx_dma_active() const;
    const TxStats& tx_stats() const;
    std::size_t service_tx_dma_sim(std::size_t max_bytes = k_dma_burst_bytes);

private:
    static constexpr std::size_t k_tx_fifo_capacity = 512;
    static constexpr std::size_t k_dma_burst_bytes = 64;

    std::array<std::uint8_t, k_tx_fifo_capacity> tx_fifo_ {};
    std::size_t tx_head_ {0};
    std::size_t tx_tail_ {0};
    std::size_t tx_count_ {0};
    std::size_t dma_inflight_bytes_ {0};
    bool tx_dma_active_ {false};
    TxStats tx_stats_ {};

    static constexpr std::size_t advance(std::size_t index);
    void start_tx_dma_if_idle();
    static constexpr std::size_t min_size(std::size_t a, std::size_t b);
};

class Stm32AnalogInput final : public IAnalogInput {
public:
    Status read_voltage(float& voltage_v) override;
};

class Stm32BusFactory final : public IBusFactory {
public:
    static constexpr std::size_t k_spi_bus_count = 2;
    static constexpr std::size_t k_spi_chip_select_count = 4;
    static constexpr std::size_t k_i2c_bus_count = 2;
    static constexpr std::size_t k_uart_port_count = 2;
    static constexpr std::size_t k_analog_channel_count = 4;

    ISpiDevice* create_spi_device(std::uint8_t bus_index,
                                  std::uint8_t chip_select_index,
                                  const SpiDeviceConfig& config) override;
    II2cDevice* create_i2c_device(std::uint8_t bus_index,
                                  const I2cDeviceConfig& config) override;
    IUartPort* create_uart_port(std::uint8_t port_index,
                                const UartConfig& config) override;
    IAnalogInput* create_analog_input(std::uint8_t channel_index) override;
    bool spi_configured(std::uint8_t bus_index, std::uint8_t chip_select_index) const;
    bool i2c_configured(std::uint8_t bus_index) const;
    bool uart_configured(std::uint8_t port_index) const;
    bool analog_configured(std::uint8_t channel_index) const;
    std::size_t service_uart_tx_dma_sim(std::size_t max_bytes_per_port);

private:
    static constexpr std::size_t k_spi_device_count =
        k_spi_bus_count * k_spi_chip_select_count;

    std::array<Stm32SpiDevice, k_spi_device_count> spi_devices_ {};
    std::array<SpiDeviceConfig, k_spi_device_count> spi_configs_ {};
    std::array<bool, k_spi_device_count> spi_configured_ {};
    std::array<Stm32I2cDevice, k_i2c_bus_count> i2c_devices_ {};
    std::array<I2cDeviceConfig, k_i2c_bus_count> i2c_configs_ {};
    std::array<bool, k_i2c_bus_count> i2c_configured_ {};
    std::array<Stm32UartPort, k_uart_port_count> uart_ports_ {};
    std::array<UartConfig, k_uart_port_count> uart_configs_ {};
    std::array<bool, k_uart_port_count> uart_configured_ {};
    std::array<Stm32AnalogInput, k_analog_channel_count> analog_inputs_ {};
    std::array<bool, k_analog_channel_count> analog_configured_ {};

    static constexpr std::size_t spi_device_index(std::uint8_t bus_index,
                                                  std::uint8_t chip_select_index);
};

} // namespace dfw::platform::stm32
