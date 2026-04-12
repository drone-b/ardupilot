#include "comms/AiControlLink.hpp"
#include "comms/TelemetryPublisher.hpp"
#include "config/Parameters.hpp"
#include "control/ControlAllocator.hpp"
#include "estimation/AttitudeEstimator.hpp"
#include "future/HardwareGraph.hpp"
#include "runtime/Scheduler.hpp"
#include "safety/SafetySupervisor.hpp"
#include "sensing/PowerMonitor.hpp"
#include "sensing/Sensor.hpp"
#include "sensing/SensorManager.hpp"
#include "platform/stm32/Stm32BusDevices.hpp"
#include "platform/stm32/Stm32Hal.hpp"
#include "platform/stm32/Stm32SensorDevices.hpp"
#include "platform/stm32/Stm32Sim.hpp"
#include "platform/stm32/Stm32TimerTimebase.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>

namespace {

bool almost_equal(float a, float b, float eps)
{
    const float diff = a - b;
    return diff < eps && diff > -eps;
}

class ManualClock final : public dfw::platform::IClock {
public:
    dfw::common::TimestampUs now_us() const override
    {
        return now_us_;
    }

    dfw::common::TimestampUs now_us_isr() const override
    {
        return now_us_;
    }

    void sleep_us(dfw::common::DurationUs duration_us) override
    {
        now_us_ += duration_us;
    }

    void set_now_us(dfw::common::TimestampUs now_us)
    {
        now_us_ = now_us;
    }

private:
    dfw::common::TimestampUs now_us_ {0};
};

class FixedAnalogInput final : public dfw::platform::IAnalogInput {
public:
    explicit FixedAnalogInput(float voltage_v) :
        voltage_v_(voltage_v)
    {
    }

    dfw::platform::Status read_voltage(float& voltage_v) override
    {
        voltage_v = voltage_v_;
        return {};
    }

private:
    float voltage_v_ {0.0f};
};

class ControlledUartPort final : public dfw::platform::IUartPort {
public:
    dfw::platform::Status write(const std::uint8_t* data, std::size_t length) override
    {
        if (data == nullptr || length == 0U) {
            return dfw::platform::Status {dfw::platform::StatusCode::invalid_argument};
        }

        last_write_size_ = length;
        ++write_count_;
        return next_status_;
    }

    std::size_t read(std::uint8_t* data, std::size_t max_length) override
    {
        (void) data;
        (void) max_length;
        return 0;
    }

    std::size_t available() const override
    {
        return 0;
    }

    void set_next_status(dfw::platform::Status status)
    {
        next_status_ = status;
    }

    std::size_t last_write_size() const
    {
        return last_write_size_;
    }

    std::uint32_t write_count() const
    {
        return write_count_;
    }

private:
    dfw::platform::Status next_status_ {};
    std::size_t last_write_size_ {0};
    std::uint32_t write_count_ {0};
};

int run_ai_link_tests()
{
    dfw::comms::AiControlLink link {};
    dfw::comms::AiWrenchCommandV1 cmd {};
    cmd.version = 1;
    cmd.seq = 10;
    cmd.t_cmd_us = 1000;
    cmd.tau[0] = 0.1f;
    cmd.tau[1] = 0.2f;
    cmd.tau[2] = 0.3f;
    cmd.thrust = 0.4f;

    if (!link.ingest_wrench(cmd, 2000, 3000)) {
        std::printf("FAIL ai_link: ingest rejected valid command\n");
        return 1;
    }

    if (!link.has_fresh_command(2500, 3000)) {
        std::printf("FAIL ai_link: fresh command not reported as fresh\n");
        return 1;
    }

    if (link.link_state(9000, 3000) != dfw::comms::AiLinkState::stale) {
        std::printf("FAIL ai_link: stale command not detected\n");
        return 1;
    }

    return 0;
}

int run_telemetry_publisher_tests()
{
    ControlledUartPort uart {};
    dfw::comms::TelemetryPublisher publisher {uart};
    dfw::comms::TelemetryFrame frame {};

    publisher.publish(frame);
    if (publisher.stats().attempted_frame_count != 1U ||
        publisher.stats().accepted_frame_count != 1U ||
        publisher.stats().busy_frame_count != 0U ||
        publisher.stats().failed_frame_count != 0U ||
        publisher.stats().last_status != dfw::platform::StatusCode::ok ||
        publisher.stats().last_write_size == 0U ||
        uart.last_write_size() != publisher.stats().last_write_size ||
        uart.write_count() != 1U) {
        std::printf("FAIL telemetry: accepted publish stats incorrect\n");
        return 1;
    }

    uart.set_next_status(dfw::platform::Status {dfw::platform::StatusCode::busy});
    publisher.publish(frame);
    if (publisher.stats().attempted_frame_count != 2U ||
        publisher.stats().accepted_frame_count != 1U ||
        publisher.stats().busy_frame_count != 1U ||
        publisher.stats().failed_frame_count != 0U ||
        publisher.stats().last_status != dfw::platform::StatusCode::busy) {
        std::printf("FAIL telemetry: busy publish stats incorrect\n");
        return 1;
    }

    uart.set_next_status(dfw::platform::Status {dfw::platform::StatusCode::io_error});
    publisher.publish(frame);
    if (publisher.stats().attempted_frame_count != 3U ||
        publisher.stats().accepted_frame_count != 1U ||
        publisher.stats().busy_frame_count != 1U ||
        publisher.stats().failed_frame_count != 1U ||
        publisher.stats().last_status != dfw::platform::StatusCode::io_error) {
        std::printf("FAIL telemetry: failure publish stats incorrect\n");
        return 1;
    }

    return 0;
}

void scheduler_overrun_task(void* context, dfw::common::TimestampUs now_us)
{
    (void) now_us;
    auto* clock = static_cast<ManualClock*>(context);
    clock->sleep_us(20);
}

int run_scheduler_tests()
{
    ManualClock clock {};
    dfw::runtime::Scheduler scheduler {clock};

    if (!scheduler.add_task("critical_test",
                            dfw::runtime::Scheduler::PriorityClass::critical_fast,
                            100,
                            10,
                            &clock,
                            scheduler_overrun_task)) {
        std::printf("FAIL scheduler: could not add task\n");
        return 1;
    }

    clock.set_now_us(100);
    scheduler.run_once();

    const dfw::runtime::Scheduler::TaskSpec* task = scheduler.task_at(0);
    if (task == nullptr || task->timing.overrun_count != 1U) {
        std::printf("FAIL scheduler: overrun not recorded\n");
        return 1;
    }

    return 0;
}

int run_parameter_tests()
{
    dfw::config::ParameterRegistry& registry = dfw::config::ParameterRegistry::instance();
    if (!registry.set_float(dfw::config::ParameterId::rate_roll_kp, 0.25f)) {
        std::printf("FAIL params: valid float rejected\n");
        return 1;
    }

    float value = 0.0f;
    if (!registry.get_float(dfw::config::ParameterId::rate_roll_kp, value) ||
        !almost_equal(value, 0.25f, 0.0001f)) {
        std::printf("FAIL params: get_float returned wrong value\n");
        return 1;
    }

    if (registry.set_float(dfw::config::ParameterId::rate_roll_kp, 50.0f)) {
        std::printf("FAIL params: out-of-range float accepted\n");
        return 1;
    }

    registry.reset_to_defaults();
    return 0;
}

int run_power_monitor_tests()
{
    FixedAnalogInput voltage_input {1.2f};
    FixedAnalogInput current_input {0.5f};
    dfw::sensing::PowerMonitorConfig config {};
    config.voltage_scale = 10.0f;
    config.current_scale = 20.0f;
    config.current_offset_v = 0.5f;

    dfw::sensing::PowerMonitor monitor {voltage_input, current_input, config};
    dfw::sensing::PowerSample sample {};
    if (!monitor.sample(1000, sample) || !sample.valid) {
        std::printf("FAIL power: valid sample rejected\n");
        return 1;
    }

    if (!almost_equal(sample.voltage_v, 12.0f, 0.001f) ||
        !almost_equal(sample.current_a, 0.0f, 0.001f)) {
        std::printf("FAIL power: scaled values incorrect\n");
        return 1;
    }

    return 0;
}

int run_safety_tests()
{
    dfw::safety::SafetySupervisor safety {};
    dfw::safety::SafetyInput input {};
    input.command_valid = true;
    input.arm_command = true;
    input.control_output_valid = true;
    input.sensor_valid = true;
    input.attitude_valid = true;
    input.watchdog_healthy = true;
    input.throttle_low = true;
    input.flight_mode_valid = true;
    input.now_us = 1000;

    dfw::safety::SafetyOutput output = safety.update(input);
    if (output.state != dfw::safety::FlightState::armed || !output.allow_motor_output) {
        std::printf("FAIL safety: valid arm command did not arm\n");
        return 1;
    }

    input.arm_command = false;
    input.sensor_valid = false;
    input.now_us = 2000;
    output = safety.update(input);
    if (output.state != dfw::safety::FlightState::armed) {
        std::printf("FAIL safety: fault debounce entered failsafe too early\n");
        return 1;
    }

    input.now_us = 102000;
    output = safety.update(input);
    if (output.state != dfw::safety::FlightState::failsafe || output.allow_motor_output) {
        std::printf("FAIL safety: debounced fault did not latch failsafe\n");
        return 1;
    }

    input.disarm_command = true;
    output = safety.update(input);
    if (output.state != dfw::safety::FlightState::disarmed ||
        output.allow_motor_output ||
        output.block_reason != dfw::safety::SafetyBlockReason::disarmed) {
        std::printf("FAIL safety: disarm did not clear failsafe latch safely\n");
        return 1;
    }

    return 0;
}

int run_stm32_hal_tests()
{
    dfw::platform::stm32::SimulatedStm32Timebase timebase {};
    dfw::platform::stm32::Stm32Clock clock {timebase};
    if (clock.now_us() != 0U) {
        std::printf("FAIL stm32_hal: simulated clock did not start at zero\n");
        return 1;
    }

    timebase.advance_us(250);
    if (clock.now_us() != 250U || clock.now_us_isr() != 250U) {
        std::printf("FAIL stm32_hal: simulated timebase did not propagate to clock\n");
        return 1;
    }

    clock.sleep_us(50);
    if (clock.now_us() != 300U) {
        std::printf("FAIL stm32_hal: simulated clock sleep did not advance time\n");
        return 1;
    }

    dfw::platform::stm32::Stm32TimerTimebase timer_timebase {1000000U};
    timer_timebase.on_timer_tick();
    timer_timebase.on_timer_tick();
    if (timer_timebase.now_us() != 2U || timer_timebase.timer_hz() != 1000000U) {
        std::printf("FAIL stm32_hal: timer timebase did not advance by microsecond ticks\n");
        return 1;
    }

    dfw::platform::stm32::Stm32BoardControl board_control {};
    board_control.service_watchdog();
    board_control.service_watchdog();
    board_control.reset_system();
    if (board_control.stats().watchdog_service_count != 2U ||
        board_control.stats().reset_request_count != 1U ||
        !board_control.reset_requested()) {
        std::printf("FAIL stm32_hal: board control stats incorrect\n");
        return 1;
    }

    dfw::platform::stm32::Stm32BusFactory bus_factory {};
    if (bus_factory.create_spi_device(0, 0, dfw::platform::SpiDeviceConfig {}) == nullptr ||
        !bus_factory.spi_configured(0, 0) ||
        bus_factory.create_spi_device(2, 0, dfw::platform::SpiDeviceConfig {}) != nullptr) {
        std::printf("FAIL stm32_hal: spi factory validation failed\n");
        return 1;
    }

    if (bus_factory.create_i2c_device(0, dfw::platform::I2cDeviceConfig {}) == nullptr ||
        !bus_factory.i2c_configured(0) ||
        bus_factory.create_i2c_device(2, dfw::platform::I2cDeviceConfig {}) != nullptr) {
        std::printf("FAIL stm32_hal: i2c factory validation failed\n");
        return 1;
    }

    dfw::platform::IUartPort* factory_uart_0 =
        bus_factory.create_uart_port(0, dfw::platform::UartConfig {});
    dfw::platform::IUartPort* factory_uart_1 =
        bus_factory.create_uart_port(1, dfw::platform::UartConfig {});
    if (factory_uart_0 == nullptr ||
        factory_uart_1 == nullptr ||
        !bus_factory.uart_configured(0) ||
        bus_factory.create_uart_port(2, dfw::platform::UartConfig {}) != nullptr) {
        std::printf("FAIL stm32_hal: uart factory validation failed\n");
        return 1;
    }

    if (bus_factory.create_analog_input(0) == nullptr ||
        !bus_factory.analog_configured(0) ||
        bus_factory.create_analog_input(4) != nullptr) {
        std::printf("FAIL stm32_hal: analog factory validation failed\n");
        return 1;
    }

    const std::uint8_t factory_uart_payload[80] {};
    if (!factory_uart_0->write(factory_uart_payload, sizeof(factory_uart_payload)).success() ||
        !factory_uart_1->write(factory_uart_payload, sizeof(factory_uart_payload)).success()) {
        std::printf("FAIL stm32_hal: factory uart write failed\n");
        return 1;
    }

    if (bus_factory.service_uart_tx_dma_sim(32U) != 64U ||
        bus_factory.service_uart_tx_dma_sim(32U) != 64U ||
        bus_factory.service_uart_tx_dma_sim(32U) != 32U ||
        bus_factory.service_uart_tx_dma_sim(32U) != 0U) {
        std::printf("FAIL stm32_hal: factory uart dma service budget incorrect\n");
        return 1;
    }

    dfw::platform::stm32::Stm32UartPort uart {};
    const std::uint8_t payload[16] {};

    if (!uart.write(payload, sizeof(payload)).success()) {
        std::printf("FAIL stm32_hal: uart rejected bounded write\n");
        return 1;
    }

    if (uart.queued_tx_bytes() != sizeof(payload)) {
        std::printf("FAIL stm32_hal: uart queued byte count incorrect\n");
        return 1;
    }

    if (!uart.tx_dma_active() || uart.dma_inflight_bytes() != sizeof(payload)) {
        std::printf("FAIL stm32_hal: uart write did not start bounded dma request\n");
        return 1;
    }

    if (uart.tx_stats().accepted_write_count != 1U ||
        uart.tx_stats().dma_start_count != 1U ||
        uart.tx_stats().total_enqueued_bytes != sizeof(payload) ||
        uart.tx_stats().max_queued_bytes != sizeof(payload)) {
        std::printf("FAIL stm32_hal: uart tx stats did not record accepted write\n");
        return 1;
    }

    const std::uint8_t large_payload[513] {};
    if (uart.write(large_payload, sizeof(large_payload)).code != dfw::platform::StatusCode::busy) {
        std::printf("FAIL stm32_hal: uart overflow did not return busy\n");
        return 1;
    }

    if (uart.tx_stats().busy_write_count != 1U) {
        std::printf("FAIL stm32_hal: uart busy stat not recorded\n");
        return 1;
    }

    if (uart.service_tx_dma_sim() != sizeof(payload) ||
        uart.queued_tx_bytes() != 0U ||
        uart.tx_dma_active()) {
        std::printf("FAIL stm32_hal: uart dma service did not drain queue\n");
        return 1;
    }

    if (uart.tx_stats().dma_complete_count != 1U ||
        uart.tx_stats().total_transmitted_bytes != sizeof(payload)) {
        std::printf("FAIL stm32_hal: uart dma stats did not record completion\n");
        return 1;
    }

    const std::uint8_t burst_payload[80] {};
    if (!uart.write(burst_payload, sizeof(burst_payload)).success() ||
        uart.dma_inflight_bytes() != 64U ||
        uart.queued_tx_bytes() != sizeof(burst_payload)) {
        std::printf("FAIL stm32_hal: uart burst write did not start bounded dma burst\n");
        return 1;
    }

    if (uart.service_tx_dma_sim(32U) != 32U ||
        uart.dma_inflight_bytes() != 32U ||
        uart.queued_tx_bytes() != 48U) {
        std::printf("FAIL stm32_hal: uart partial dma service incorrect\n");
        return 1;
    }

    if (uart.service_tx_dma_sim() != 32U ||
        uart.dma_inflight_bytes() != 16U ||
        uart.queued_tx_bytes() != 16U ||
        !uart.tx_dma_active()) {
        std::printf("FAIL stm32_hal: uart dma did not continue to next burst\n");
        return 1;
    }

    if (uart.service_tx_dma_sim() != 16U ||
        uart.queued_tx_bytes() != 0U ||
        uart.tx_dma_active()) {
        std::printf("FAIL stm32_hal: uart final dma burst did not complete\n");
        return 1;
    }

    if (uart.tx_stats().dma_start_count != 3U ||
        uart.tx_stats().dma_complete_count != 3U ||
        uart.tx_stats().total_transmitted_bytes != sizeof(payload) + sizeof(burst_payload) ||
        uart.tx_stats().max_queued_bytes != sizeof(burst_payload)) {
        std::printf("FAIL stm32_hal: uart burst dma stats incorrect\n");
        return 1;
    }

    dfw::platform::stm32::Stm32PwmBank pwm {};
    if (pwm.channel_count() != 8U) {
        std::printf("FAIL stm32_hal: pwm channel count incorrect\n");
        return 1;
    }

    if (pwm.write_normalized(0, 0.5f).code != dfw::platform::StatusCode::invalid_argument) {
        std::printf("FAIL stm32_hal: pwm accepted write before configure\n");
        return 1;
    }

    if (!pwm.configure_channel(0, dfw::platform::PwmChannelConfig {}).success() ||
        !pwm.channel_configured(0)) {
        std::printf("FAIL stm32_hal: pwm configure failed\n");
        return 1;
    }

    if (!pwm.write_normalized(0, 1.5f).success() ||
        !almost_equal(pwm.channel_value(0), 1.0f, 0.0001f)) {
        std::printf("FAIL stm32_hal: pwm write did not clamp high command\n");
        return 1;
    }

    pwm.disarm_all();
    if (!almost_equal(pwm.channel_value(0), 0.0f, 0.0001f)) {
        std::printf("FAIL stm32_hal: pwm disarm did not zero output\n");
        return 1;
    }

    dfw::platform::stm32::Stm32SpiDevice spi {};
    const std::uint8_t tx_data[4] {1U, 2U, 3U, 4U};
    std::uint8_t rx_data[4] {};
    if (!spi.transfer(tx_data, rx_data, sizeof(tx_data)).success() ||
        rx_data[0] != 1U ||
        rx_data[3] != 4U) {
        std::printf("FAIL stm32_hal: spi bounded transfer failed\n");
        return 1;
    }

    if (spi.transfer_stats().accepted_transfer_count != 1U ||
        spi.transfer_stats().total_transferred_bytes != sizeof(tx_data)) {
        std::printf("FAIL stm32_hal: spi stats did not record accepted transfer\n");
        return 1;
    }

    spi.set_busy(true);
    if (spi.transfer(tx_data, rx_data, sizeof(tx_data)).code != dfw::platform::StatusCode::busy ||
        spi.transfer_stats().busy_transfer_count != 1U) {
        std::printf("FAIL stm32_hal: spi busy transfer not reported\n");
        return 1;
    }

    std::uint8_t large_rx[65] {};
    const std::uint8_t large_tx[65] {};
    spi.set_busy(false);
    if (spi.transfer(large_tx, large_rx, sizeof(large_tx)).code !=
        dfw::platform::StatusCode::invalid_argument) {
        std::printf("FAIL stm32_hal: spi oversize transfer not rejected\n");
        return 1;
    }

    dfw::platform::stm32::Stm32SensorInterruptRouter router {};
    std::uint32_t irq_count = 0;
    router.register_data_ready_line(
        0,
        [](void* context) {
            auto* count = static_cast<std::uint32_t*>(context);
            ++(*count);
        },
        &irq_count);

    if (!router.simulate_data_ready(0) || irq_count != 1U) {
        std::printf("FAIL stm32_hal: sensor irq route did not call callback\n");
        return 1;
    }

    if (router.simulate_data_ready(7)) {
        std::printf("FAIL stm32_hal: unregistered irq line reported success\n");
        return 1;
    }

    dfw::platform::stm32::Stm32I2cDevice i2c {};
    const std::uint8_t i2c_tx[2] {0x10U, 0x20U};
    std::uint8_t i2c_rx[4] {};
    if (!i2c.write(i2c_tx, sizeof(i2c_tx)).success() ||
        !i2c.write_read(i2c_tx, sizeof(i2c_tx), i2c_rx, sizeof(i2c_rx)).success() ||
        i2c_rx[0] != 0x10U ||
        i2c_rx[1] != 0x20U ||
        i2c_rx[2] != 0x10U) {
        std::printf("FAIL stm32_hal: i2c bounded transfer failed\n");
        return 1;
    }

    if (i2c.transfer_stats().accepted_transfer_count != 2U ||
        i2c.transfer_stats().total_written_bytes != 4U ||
        i2c.transfer_stats().total_read_bytes != 4U) {
        std::printf("FAIL stm32_hal: i2c stats incorrect\n");
        return 1;
    }

    i2c.set_busy(true);
    if (i2c.write(i2c_tx, sizeof(i2c_tx)).code != dfw::platform::StatusCode::busy ||
        i2c.transfer_stats().busy_transfer_count != 1U) {
        std::printf("FAIL stm32_hal: i2c busy transfer not reported\n");
        return 1;
    }
    i2c.set_busy(false);

    dfw::platform::stm32::Stm32BarometerDevice barometer {i2c};
    if (!barometer.initialize()) {
        std::printf("FAIL stm32_hal: barometer failed to initialize\n");
        return 1;
    }

    dfw::sensing::BarometerSample baro_sample {};
    if (!barometer.read_sample(clock.now_us(), clock.now_us() + 50U, baro_sample) ||
        !baro_sample.valid ||
        baro_sample.status != dfw::sensing::BarometerSampleStatus::ok ||
        !almost_equal(baro_sample.pressure_pa, 101325.0f, 0.1f) ||
        baro_sample.transport_latency_us != 50U) {
        std::printf("FAIL stm32_hal: barometer sample invalid\n");
        return 1;
    }

    dfw::platform::stm32::Stm32MagnetometerDevice magnetometer {i2c};
    if (!magnetometer.initialize()) {
        std::printf("FAIL stm32_hal: magnetometer failed to initialize\n");
        return 1;
    }

    dfw::sensing::MagnetometerSample mag_sample {};
    if (!magnetometer.read_sample(clock.now_us(), clock.now_us() + 25U, mag_sample) ||
        !mag_sample.valid ||
        mag_sample.status != dfw::sensing::MagnetometerSampleStatus::ok ||
        !almost_equal(mag_sample.magnetic_field_ut[0], 20.0f, 0.001f) ||
        !almost_equal(mag_sample.magnetic_field_ut[2], 45.0f, 0.001f) ||
        mag_sample.transport_latency_us != 25U) {
        std::printf("FAIL stm32_hal: magnetometer sample invalid\n");
        return 1;
    }

    dfw::platform::stm32::Stm32ImuDevice imu {spi};
    dfw::sensing::SensorManager sensor_manager {imu};
    if (!sensor_manager.initialize()) {
        std::printf("FAIL stm32_hal: stm32 imu failed to initialize\n");
        return 1;
    }

    struct IrqContext {
        dfw::platform::stm32::Stm32Clock* clock {nullptr};
        dfw::sensing::SensorManager* sensor_manager {nullptr};
    } irq_context {&clock, &sensor_manager};

    router.register_data_ready_line(
        1,
        [](void* context) {
            auto* irq = static_cast<IrqContext*>(context);
            irq->sensor_manager->notify_imu_trigger(irq->clock->now_us_isr());
        },
        &irq_context);

    timebase.advance_us(1000);
    if (!router.simulate_data_ready(1)) {
        std::printf("FAIL stm32_hal: imu data-ready irq did not fire\n");
        return 1;
    }

    timebase.advance_us(100);
    if (!sensor_manager.acquire_imu(clock.now_us())) {
        std::printf("FAIL stm32_hal: sensor manager did not acquire stm32 imu sample\n");
        return 1;
    }

    dfw::sensing::ImuSample imu_sample {};
    if (!sensor_manager.get_latest_imu_sample(imu_sample) ||
        !imu_sample.valid ||
        imu_sample.status != dfw::sensing::ImuSampleStatus::ok ||
        imu_sample.transport_latency_us != 100U) {
        std::printf("FAIL stm32_hal: latest stm32 imu sample invalid\n");
        return 1;
    }

    dfw::estimation::AttitudeEstimator estimator {};
    if (!estimator.update(imu_sample) || !estimator.state().valid) {
        std::printf("FAIL stm32_hal: estimator rejected stm32 imu sample\n");
        return 1;
    }

    return 0;
}

int run_estimator_tests()
{
    dfw::estimation::AttitudeEstimator estimator {};
    dfw::sensing::ImuSample sample {};
    sample.sample_time_us = 2000;
    sample.sample_period_us = 2000;
    sample.accel_mps2[0] = 0.0f;
    sample.accel_mps2[1] = 0.0f;
    sample.accel_mps2[2] = -9.81f;
    sample.gyro_rad_s[0] = 0.0f;
    sample.gyro_rad_s[1] = 0.0f;
    sample.gyro_rad_s[2] = 0.0f;
    sample.valid = true;
    sample.status = dfw::sensing::ImuSampleStatus::ok;

    if (!estimator.update(sample)) {
        std::printf("FAIL estimator: valid sample rejected\n");
        return 1;
    }

    const dfw::estimation::AttitudeState state = estimator.state();
    const float q_norm =
        std::sqrt(state.q[0] * state.q[0] + state.q[1] * state.q[1] + state.q[2] * state.q[2] +
                  state.q[3] * state.q[3]);

    if (!almost_equal(q_norm, 1.0f, 0.01f)) {
        std::printf("FAIL estimator: quaternion norm out of bounds\n");
        return 1;
    }

    for (float covariance : state.covariance_diag) {
        if (covariance <= 0.0f || covariance > 4.0f) {
            std::printf("FAIL estimator: covariance diagonal out of bounds\n");
            return 1;
        }
    }

    return 0;
}

int run_allocator_tests()
{
    dfw::control::ControlAllocator allocator {};
    dfw::control::ControlDemand demand {};
    demand.roll = 0.8f;
    demand.pitch = 0.6f;
    demand.yaw = 0.2f;
    demand.thrust = 1.5f;

    const dfw::control::MotorOutputs outputs = allocator.allocate(demand);
    for (float value : outputs.values) {
        if (value < 0.0f || value > 1.0f) {
            std::printf("FAIL allocator: output out of [0,1]\n");
            return 1;
        }
    }

    return 0;
}

int run_hardware_graph_tests()
{
    dfw::future::HardwareGraph graph_a {};
    dfw::future::HardwareGraph graph_b {};

    const dfw::future::NodeDescriptor imu_node {
        1001,
        0xA1B2C3D4U,
        1,
        10,
        0,
        dfw::future::BusKind::spi,
        true,
    };
    const dfw::future::NodeDescriptor esc_node {
        1002,
        0xAA55AA55U,
        2,
        20,
        0,
        dfw::future::BusKind::dronecan,
        true,
    };

    (void) graph_a.add_node(imu_node);
    (void) graph_a.add_node(esc_node);
    (void) graph_b.add_node(esc_node);
    (void) graph_b.add_node(imu_node);

    const auto snap_a = graph_a.snapshot();
    const auto snap_b = graph_b.snapshot();

    if (snap_a.fingerprint_crc32 != snap_b.fingerprint_crc32) {
        std::printf("FAIL hw_graph: fingerprint not deterministic\n");
        return 1;
    }

    return 0;
}

} // namespace

int main()
{
    int failures = 0;
    failures += run_ai_link_tests();
    failures += run_telemetry_publisher_tests();
    failures += run_scheduler_tests();
    failures += run_parameter_tests();
    failures += run_power_monitor_tests();
    failures += run_safety_tests();
    failures += run_stm32_hal_tests();
    failures += run_estimator_tests();
    failures += run_allocator_tests();
    failures += run_hardware_graph_tests();

    if (failures != 0) {
        std::printf("Regression suite failed: %d test group(s)\n", failures);
        return 1;
    }

    std::printf("Regression suite passed\n");
    return 0;
}
