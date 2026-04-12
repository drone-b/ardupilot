#include "platform/stm32/Stm32BusDevices.hpp"

namespace dfw::platform::stm32 {

namespace {

Status unsupported()
{
    return Status {StatusCode::unsupported};
}

} // namespace

Status Stm32SpiDevice::transfer(const std::uint8_t* tx_data,
                                std::uint8_t* rx_data,
                                std::size_t length)
{
    if (tx_data == nullptr || rx_data == nullptr || length == 0U ||
        length > k_max_transfer_bytes) {
        ++transfer_stats_.invalid_transfer_count;
        return Status {StatusCode::invalid_argument};
    }

    if (busy_) {
        ++transfer_stats_.busy_transfer_count;
        return Status {StatusCode::busy};
    }

    busy_ = true;
    for (std::size_t index = 0; index < length; ++index) {
        rx_data[index] = tx_data[index];
    }
    busy_ = false;

    ++transfer_stats_.accepted_transfer_count;
    transfer_stats_.total_transferred_bytes += static_cast<std::uint32_t>(length);
    return {};
}

const Stm32SpiDevice::TransferStats& Stm32SpiDevice::transfer_stats() const
{
    return transfer_stats_;
}

void Stm32SpiDevice::set_busy(bool busy)
{
    busy_ = busy;
}

Status Stm32I2cDevice::write(const std::uint8_t* data, std::size_t length)
{
    if (data == nullptr || length == 0U || length > k_max_write_bytes) {
        ++transfer_stats_.invalid_transfer_count;
        return Status {StatusCode::invalid_argument};
    }

    if (busy_) {
        ++transfer_stats_.busy_transfer_count;
        return Status {StatusCode::busy};
    }

    ++transfer_stats_.accepted_transfer_count;
    transfer_stats_.total_written_bytes += static_cast<std::uint32_t>(length);
    return {};
}

Status Stm32I2cDevice::write_read(const std::uint8_t* tx_data,
                                  std::size_t tx_length,
                                  std::uint8_t* rx_data,
                                  std::size_t rx_length)
{
    if (tx_data == nullptr || rx_data == nullptr ||
        tx_length == 0U || rx_length == 0U ||
        tx_length > k_max_write_bytes || rx_length > k_max_read_bytes) {
        ++transfer_stats_.invalid_transfer_count;
        return Status {StatusCode::invalid_argument};
    }

    if (busy_) {
        ++transfer_stats_.busy_transfer_count;
        return Status {StatusCode::busy};
    }

    for (std::size_t index = 0; index < rx_length; ++index) {
        rx_data[index] = tx_data[index % tx_length];
    }

    ++transfer_stats_.accepted_transfer_count;
    transfer_stats_.total_written_bytes += static_cast<std::uint32_t>(tx_length);
    transfer_stats_.total_read_bytes += static_cast<std::uint32_t>(rx_length);
    return {};
}

const Stm32I2cDevice::TransferStats& Stm32I2cDevice::transfer_stats() const
{
    return transfer_stats_;
}

void Stm32I2cDevice::set_busy(bool busy)
{
    busy_ = busy;
}

Status Stm32UartPort::write(const std::uint8_t* data, std::size_t length)
{
    if (data == nullptr || length == 0U) {
        ++tx_stats_.invalid_write_count;
        return Status {StatusCode::invalid_argument};
    }

    if (length > (k_tx_fifo_capacity - tx_count_)) {
        ++tx_stats_.busy_write_count;
        return Status {StatusCode::busy};
    }

    for (std::size_t index = 0; index < length; ++index) {
        tx_fifo_[tx_head_] = data[index];
        tx_head_ = advance(tx_head_);
    }

    tx_count_ += length;
    ++tx_stats_.accepted_write_count;
    tx_stats_.total_enqueued_bytes += static_cast<std::uint32_t>(length);
    if (tx_count_ > tx_stats_.max_queued_bytes) {
        tx_stats_.max_queued_bytes = static_cast<std::uint16_t>(tx_count_);
    }
    start_tx_dma_if_idle();
    return {};
}

std::size_t Stm32UartPort::read(std::uint8_t* data, std::size_t max_length)
{
    (void) data;
    (void) max_length;
    return 0;
}

std::size_t Stm32UartPort::available() const
{
    return 0;
}

std::size_t Stm32UartPort::queued_tx_bytes() const
{
    return tx_count_;
}

std::size_t Stm32UartPort::dma_inflight_bytes() const
{
    return dma_inflight_bytes_;
}

bool Stm32UartPort::tx_dma_active() const
{
    return tx_dma_active_;
}

const Stm32UartPort::TxStats& Stm32UartPort::tx_stats() const
{
    return tx_stats_;
}

std::size_t Stm32UartPort::service_tx_dma_sim(std::size_t max_bytes)
{
    if (!tx_dma_active_ || dma_inflight_bytes_ == 0U || max_bytes == 0U) {
        return 0;
    }

    const std::size_t transmitted_bytes = min_size(max_bytes, dma_inflight_bytes_);
    for (std::size_t index = 0; index < transmitted_bytes; ++index) {
        tx_tail_ = advance(tx_tail_);
    }

    tx_count_ -= transmitted_bytes;
    dma_inflight_bytes_ -= transmitted_bytes;
    tx_stats_.total_transmitted_bytes += static_cast<std::uint32_t>(transmitted_bytes);

    if (dma_inflight_bytes_ == 0U) {
        tx_dma_active_ = false;
        ++tx_stats_.dma_complete_count;
        start_tx_dma_if_idle();
    }

    return transmitted_bytes;
}

constexpr std::size_t Stm32UartPort::advance(std::size_t index)
{
    return (index + 1U) % k_tx_fifo_capacity;
}

void Stm32UartPort::start_tx_dma_if_idle()
{
    if (tx_dma_active_ || tx_count_ == 0U) {
        return;
    }

    dma_inflight_bytes_ = min_size(tx_count_, k_dma_burst_bytes);
    tx_dma_active_ = true;
    ++tx_stats_.dma_start_count;
}

constexpr std::size_t Stm32UartPort::min_size(std::size_t a, std::size_t b)
{
    return a < b ? a : b;
}

Status Stm32AnalogInput::read_voltage(float& voltage_v)
{
    voltage_v = 0.0f;
    return unsupported();
}

ISpiDevice* Stm32BusFactory::create_spi_device(std::uint8_t bus_index,
                                               std::uint8_t chip_select_index,
                                               const SpiDeviceConfig& config)
{
    if (bus_index >= k_spi_bus_count ||
        chip_select_index >= k_spi_chip_select_count ||
        config.bus_hz == 0U) {
        return nullptr;
    }

    const std::size_t index = spi_device_index(bus_index, chip_select_index);
    spi_configs_[index] = config;
    spi_configured_[index] = true;
    return &spi_devices_[index];
}

II2cDevice* Stm32BusFactory::create_i2c_device(std::uint8_t bus_index,
                                               const I2cDeviceConfig& config)
{
    if (bus_index >= k_i2c_bus_count || config.bus_hz == 0U || config.address > 0x7FU) {
        return nullptr;
    }

    i2c_configs_[bus_index] = config;
    i2c_configured_[bus_index] = true;
    return &i2c_devices_[bus_index];
}

IUartPort* Stm32BusFactory::create_uart_port(std::uint8_t port_index,
                                             const UartConfig& config)
{
    if (port_index >= k_uart_port_count || config.baud_rate == 0U) {
        return nullptr;
    }

    uart_configs_[port_index] = config;
    uart_configured_[port_index] = true;
    return &uart_ports_[port_index];
}

IAnalogInput* Stm32BusFactory::create_analog_input(std::uint8_t channel_index)
{
    if (channel_index >= k_analog_channel_count) {
        return nullptr;
    }

    analog_configured_[channel_index] = true;
    return &analog_inputs_[channel_index];
}

bool Stm32BusFactory::spi_configured(std::uint8_t bus_index,
                                     std::uint8_t chip_select_index) const
{
    if (bus_index >= k_spi_bus_count || chip_select_index >= k_spi_chip_select_count) {
        return false;
    }

    return spi_configured_[spi_device_index(bus_index, chip_select_index)];
}

bool Stm32BusFactory::i2c_configured(std::uint8_t bus_index) const
{
    if (bus_index >= k_i2c_bus_count) {
        return false;
    }

    return i2c_configured_[bus_index];
}

bool Stm32BusFactory::uart_configured(std::uint8_t port_index) const
{
    if (port_index >= k_uart_port_count) {
        return false;
    }

    return uart_configured_[port_index];
}

bool Stm32BusFactory::analog_configured(std::uint8_t channel_index) const
{
    if (channel_index >= k_analog_channel_count) {
        return false;
    }

    return analog_configured_[channel_index];
}

std::size_t Stm32BusFactory::service_uart_tx_dma_sim(std::size_t max_bytes_per_port)
{
    std::size_t transmitted_bytes = 0;
    for (std::size_t index = 0; index < k_uart_port_count; ++index) {
        transmitted_bytes += uart_ports_[index].service_tx_dma_sim(max_bytes_per_port);
    }

    return transmitted_bytes;
}

constexpr std::size_t Stm32BusFactory::spi_device_index(std::uint8_t bus_index,
                                                        std::uint8_t chip_select_index)
{
    return (static_cast<std::size_t>(bus_index) * k_spi_chip_select_count) +
           static_cast<std::size_t>(chip_select_index);
}

} // namespace dfw::platform::stm32
