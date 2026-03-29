#include "comms/AiControlLink.hpp"
#include "comms/TelemetryPublisher.hpp"
#include "control/ControlAllocator.hpp"
#include "control/ControlLoop.hpp"
#include "control/FlightModeManager.hpp"
#include "estimation/AttitudeEstimator.hpp"
#include "logging/Logger.hpp"
#include "logging/SdLogStorage.hpp"
#include "platform/Hal.hpp"
#include "runtime/Scheduler.hpp"
#include "safety/SafetySupervisor.hpp"
#include "sensing/SensorManager.hpp"

#include <chrono>
#include <cstddef>
#include <thread>

namespace {

constexpr dfw::common::DurationUs k_command_timeout_us = 100000;
constexpr dfw::common::DurationUs k_major_cycle_us = 2000;
constexpr dfw::common::DurationUs k_ai_command_max_latency_us = 25000;

class SimClock final : public dfw::platform::IClock {
public:
    SimClock() :
        start_(std::chrono::steady_clock::now())
    {
    }

    dfw::common::TimestampUs now_us() const override
    {
        const auto elapsed = std::chrono::steady_clock::now() - start_;
        return static_cast<dfw::common::TimestampUs>(
            std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count());
    }

    dfw::common::TimestampUs now_us_isr() const override
    {
        return now_us();
    }

    void sleep_us(dfw::common::DurationUs duration_us) override
    {
        std::this_thread::sleep_for(std::chrono::microseconds(duration_us));
    }

private:
    std::chrono::steady_clock::time_point start_;
};

class SimBoardControl final : public dfw::platform::IBoardControl {
public:
    void service_watchdog() override
    {
    }

    void reset_system() override
    {
    }
};

class SimOutputDriver final : public dfw::platform::IOutputDriver {
public:
    void write_channel(std::uint8_t channel, float normalized_command) override
    {
        (void) channel;
        (void) normalized_command;
    }
};

class SimSpiDevice final : public dfw::platform::ISpiDevice {
public:
    dfw::platform::Status transfer(const std::uint8_t* tx_data,
                                   std::uint8_t* rx_data,
                                   std::size_t length) override
    {
        (void) tx_data;
        (void) rx_data;
        (void) length;
        return {};
    }
};

class SimI2cDevice final : public dfw::platform::II2cDevice {
public:
    dfw::platform::Status write(const std::uint8_t* data, std::size_t length) override
    {
        (void) data;
        (void) length;
        return {};
    }

    dfw::platform::Status write_read(const std::uint8_t* tx_data,
                                     std::size_t tx_length,
                                     std::uint8_t* rx_data,
                                     std::size_t rx_length) override
    {
        (void) tx_data;
        (void) tx_length;
        (void) rx_data;
        (void) rx_length;
        return {};
    }
};

class SimUartPort final : public dfw::platform::IUartPort {
public:
    dfw::platform::Status write(const std::uint8_t* data, std::size_t length) override
    {
        (void) data;
        (void) length;
        return {};
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
};

class SimBusFactory final : public dfw::platform::IBusFactory {
public:
    dfw::platform::ISpiDevice* create_spi_device(std::uint8_t bus_index,
                                                 std::uint8_t chip_select_index,
                                                 const dfw::platform::SpiDeviceConfig& config) override
    {
        (void) bus_index;
        (void) chip_select_index;
        (void) config;
        return &spi_device_;
    }

    dfw::platform::II2cDevice* create_i2c_device(std::uint8_t bus_index,
                                                 const dfw::platform::I2cDeviceConfig& config) override
    {
        (void) bus_index;
        (void) config;
        return &i2c_device_;
    }

    dfw::platform::IUartPort* create_uart_port(std::uint8_t port_index,
                                               const dfw::platform::UartConfig& config) override
    {
        (void) port_index;
        (void) config;
        return &uart_port_;
    }

private:
    SimSpiDevice spi_device_ {};
    SimI2cDevice i2c_device_ {};
    SimUartPort uart_port_ {};
};

class SimPwmBank final : public dfw::platform::IPwmBank {
public:
    std::size_t channel_count() const override
    {
        return 8;
    }

    dfw::platform::Status configure_channel(std::uint8_t channel,
                                            const dfw::platform::PwmChannelConfig& config) override
    {
        (void) channel;
        (void) config;
        return {};
    }

    dfw::platform::Status write_normalized(std::uint8_t channel, float normalized_command) override
    {
        (void) channel;
        (void) normalized_command;
        return {};
    }

    void disarm_all() override
    {
    }
};

class SimSensorInterruptRouter final : public dfw::platform::ISensorInterruptRouter {
public:
    void register_data_ready_line(std::uint8_t line_index,
                                  void (*callback)(void*),
                                  void* context) override
    {
        (void) line_index;
        (void) callback;
        (void) context;
    }
};

class SimAppendStorageDevice final : public dfw::logging::IAppendStorageDevice {
public:
    bool initialize() override
    {
        initialized_ = true;
        return true;
    }

    bool append_bytes(const std::uint8_t* data, std::size_t length) override
    {
        (void) data;
        (void) length;
        return initialized_;
    }

private:
    bool initialized_ {false};
};

class SimHal final : public dfw::platform::IHal {
public:
    dfw::platform::IClock& clock() override
    {
        return clock_;
    }

    dfw::platform::IBoardControl& board_control() override
    {
        return board_control_;
    }

    dfw::platform::IOutputDriver& output_driver() override
    {
        return output_driver_;
    }

    dfw::platform::IBusFactory& buses() override
    {
        return bus_factory_;
    }

    dfw::platform::IPwmBank& pwm_bank() override
    {
        return pwm_bank_;
    }

    dfw::platform::ISensorInterruptRouter& sensor_interrupts() override
    {
        return sensor_interrupt_router_;
    }

private:
    SimClock clock_ {};
    SimBoardControl board_control_ {};
    SimOutputDriver output_driver_ {};
    SimBusFactory bus_factory_ {};
    SimPwmBank pwm_bank_ {};
    SimSensorInterruptRouter sensor_interrupt_router_ {};
};

class SimImuDevice final : public dfw::sensing::IImuDevice {
public:
    explicit SimImuDevice(dfw::platform::ISpiDevice& spi_device) :
        spi_device_(spi_device)
    {
    }

    bool initialize() override
    {
        return true;
    }

    bool read_sample(dfw::common::TimestampUs trigger_time_us,
                     dfw::common::TimestampUs acquisition_time_us,
                     dfw::sensing::ImuSample& out_sample) override
    {
        std::uint8_t tx_buffer[12] {};
        std::uint8_t rx_buffer[12] {};

        if (!spi_device_.transfer(tx_buffer, rx_buffer, sizeof(tx_buffer)).success()) {
            return false;
        }

        out_sample.sample_time_us = trigger_time_us;
        out_sample.acquisition_time_us = acquisition_time_us;
        out_sample.transport_latency_us =
            static_cast<dfw::common::DurationUs>(acquisition_time_us - trigger_time_us);
        out_sample.sample_period_us = 0;
        out_sample.accel_mps2[0] = 0.0f;
        out_sample.accel_mps2[1] = 0.0f;
        out_sample.accel_mps2[2] = -9.81f;
        out_sample.gyro_rad_s[0] = 0.0f;
        out_sample.gyro_rad_s[1] = 0.0f;
        out_sample.gyro_rad_s[2] = 0.0f;
        out_sample.temperature_c = 25.0f;
        out_sample.status = dfw::sensing::ImuSampleStatus::ok;
        return true;
    }

private:
    dfw::platform::ISpiDevice& spi_device_;
};

struct ImuIrqContext {
    dfw::platform::IClock* clock {nullptr};
    dfw::sensing::SensorManager* sensor_manager {nullptr};
    dfw::runtime::Scheduler* scheduler {nullptr};
};

void imu_irq_callback(void* context)
{
    auto* irq_context = static_cast<ImuIrqContext*>(context);
    const dfw::common::TimestampUs trigger_time_us = irq_context->clock->now_us_isr();
    irq_context->sensor_manager->notify_imu_trigger(trigger_time_us);
    irq_context->scheduler->notify_event("imu_acquisition");
}

bool flight_mode_is_valid(dfw::control::FlightMode mode)
{
    switch (mode) {
    case dfw::control::FlightMode::stabilize:
    case dfw::control::FlightMode::acro:
        return true;
    }

    return false;
}

dfw::control::PilotCommand make_demo_pilot_command(dfw::common::TimestampUs timestamp_us)
{
    return dfw::control::PilotCommand {
        {
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            true,
        },
        dfw::control::FlightMode::stabilize,
        true,
        false,
        true,
        timestamp_us,
    };
}

dfw::comms::AiWrenchCommandV1 make_demo_ai_command(dfw::common::TimestampUs timestamp_us)
{
    dfw::comms::AiWrenchCommandV1 command {};
    command.version = 1;
    command.seq = static_cast<std::uint32_t>(timestamp_us & 0xFFFFFFFFU);
    command.t_cmd_us = timestamp_us;
    command.tau[0] = 0.0f;
    command.tau[1] = 0.0f;
    command.tau[2] = 0.0f;
    command.thrust = 0.0f;
    return command;
}

bool command_is_fresh(const dfw::control::PilotCommand& command,
                      dfw::common::TimestampUs now_us)
{
    if (!command.valid) {
        return false;
    }

    if (!command.input.valid) {
        return false;
    }

    if (command.timestamp_us == 0) {
        return false;
    }

    if (now_us < command.timestamp_us) {
        return false;
    }

    return (now_us - command.timestamp_us) <= k_command_timeout_us;
}

using FlightLogger = dfw::logging::Logger<32>;

struct AppContext {
    SimHal* hal {nullptr};
    dfw::runtime::Scheduler* scheduler {nullptr};
    dfw::control::ControlLoop* control_loop {nullptr};
    dfw::control::FlightModeManager* flight_mode_manager {nullptr};
    dfw::control::ControlAllocator* allocator {nullptr};
    dfw::comms::AiControlLink* ai_control_link {nullptr};
    dfw::comms::TelemetryPublisher* telemetry_publisher {nullptr};
    FlightLogger* logger {nullptr};
    dfw::estimation::AttitudeEstimator* attitude_estimator {nullptr};
    dfw::safety::SafetySupervisor* safety_supervisor {nullptr};
    dfw::sensing::SensorManager* sensor_manager {nullptr};
    dfw::sensing::ImuSample* latest_imu_sample {nullptr};
    dfw::control::MotorOutputs* latest_motor_outputs {nullptr};
    dfw::control::AllocatorStatus* latest_allocator_status {nullptr};
    dfw::safety::SafetyOutput* latest_safety_output {nullptr};
    dfw::control::PilotCommand* pilot_command {nullptr};
};

void imu_acquisition_task(void* context, dfw::common::TimestampUs now_us)
{
    auto* app = static_cast<AppContext*>(context);
    app->sensor_manager->acquire_imu(now_us);
}

void control_update_task(void* context, dfw::common::TimestampUs now_us)
{
    auto* app = static_cast<AppContext*>(context);

    const dfw::comms::AiWrenchCommandV1 ai_command = make_demo_ai_command(now_us);
    (void) app->ai_control_link->ingest_wrench(ai_command, now_us, k_ai_command_max_latency_us);

    *app->pilot_command = make_demo_pilot_command(now_us);

    if (!app->latest_imu_sample->valid ||
        app->latest_imu_sample->status != dfw::sensing::ImuSampleStatus::ok ||
        !app->attitude_estimator->state().valid) {
        return;
    }

    dfw::control::MotionTarget target {};
    bool target_valid = false;

    if (app->ai_control_link->has_fresh_command(now_us, k_ai_command_max_latency_us)) {
        const dfw::comms::AiWrenchCommandV1& fresh_ai = app->ai_control_link->latest_wrench();
        // Current control loop is rate-centric; this maps wrench contract into
        // rate targets as an interim compatibility path.
        target.roll_rate_target_rad_s = fresh_ai.tau[0];
        target.pitch_rate_target_rad_s = fresh_ai.tau[1];
        target.yaw_rate_target_rad_s = fresh_ai.tau[2];
        target.thrust_target = fresh_ai.thrust;
        target_valid = true;
    } else {
        const dfw::control::RateTargetSetpoint rate_target =
            app->flight_mode_manager->update(*app->pilot_command, app->attitude_estimator->state());
        if (rate_target.valid) {
            target.roll_rate_target_rad_s = rate_target.roll_rate_target_rad_s;
            target.pitch_rate_target_rad_s = rate_target.pitch_rate_target_rad_s;
            target.yaw_rate_target_rad_s = rate_target.yaw_rate_target_rad_s;
            target.thrust_target = rate_target.thrust_target;
            target_valid = true;
        }
    }

    if (!target_valid) {
        return;
    }

    app->control_loop->update(*app->latest_imu_sample, target, now_us);
}

void estimation_update_task(void* context, dfw::common::TimestampUs now_us)
{
    (void) now_us;
    auto* app = static_cast<AppContext*>(context);

    dfw::sensing::ImuSample imu_sample {};
    if (!app->sensor_manager->get_latest_imu_sample(imu_sample)) {
        return;
    }

    *app->latest_imu_sample = imu_sample;
    (void) app->attitude_estimator->update(imu_sample);
}

void output_update_task(void* context, dfw::common::TimestampUs now_us)
{
    auto* app = static_cast<AppContext*>(context);
    const bool command_valid =
        app->ai_control_link->has_fresh_command(now_us, k_ai_command_max_latency_us) ||
        command_is_fresh(*app->pilot_command, now_us);
    const bool sensor_valid =
        app->latest_imu_sample->valid &&
        app->latest_imu_sample->status == dfw::sensing::ImuSampleStatus::ok;
    const bool attitude_valid = app->attitude_estimator->state().valid;
    const bool flight_mode_valid = flight_mode_is_valid(app->pilot_command->mode);
    const bool throttle_low = command_valid && app->pilot_command->input.throttle <= 0.10f;

    dfw::safety::SafetyInput safety_input {};
    safety_input.command_valid = command_valid;
    safety_input.arm_command = app->pilot_command->arm;
    safety_input.disarm_command = app->pilot_command->disarm;
    safety_input.control_output_valid = app->control_loop->latest_result().output_valid;
    safety_input.sensor_valid = sensor_valid;
    safety_input.attitude_valid = attitude_valid;
    safety_input.watchdog_healthy = true;
    safety_input.throttle_low = throttle_low;
    safety_input.flight_mode_valid = flight_mode_valid;
    safety_input.now_us = now_us;

    *app->latest_safety_output = app->safety_supervisor->update(safety_input);
    if (!app->latest_safety_output->allow_motor_output) {
        *app->latest_motor_outputs = {};
        app->hal->pwm_bank().disarm_all();
        return;
    }

    const dfw::control::ControlDemand& demand = app->control_loop->latest_demand();
    const dfw::control::MotorOutputs motor_outputs = app->allocator->allocate(demand);
    *app->latest_motor_outputs = motor_outputs;
    *app->latest_allocator_status = app->allocator->last_status();

    for (std::uint8_t motor_index = 0; motor_index < 4; ++motor_index) {
        app->hal->pwm_bank().write_normalized(motor_index, motor_outputs.values[motor_index]);
    }
}

void telemetry_publish_task(void* context, dfw::common::TimestampUs now_us)
{
    auto* app = static_cast<AppContext*>(context);
    const dfw::comms::TelemetryFrame telemetry_frame {
        now_us,
        app->attitude_estimator->state(),
        *app->latest_imu_sample,
        app->control_loop->latest_demand(),
        app->control_loop->latest_debug(),
        *app->latest_motor_outputs,
        *app->latest_allocator_status,
        static_cast<std::uint8_t>(app->scheduler->mode()),
        app->latest_safety_output->state,
    };
    app->telemetry_publisher->publish(telemetry_frame);
    app->logger->push(telemetry_frame);
}

void log_flush_task(void* context, dfw::common::TimestampUs now_us)
{
    (void) now_us;
    auto* app = static_cast<AppContext*>(context);
    app->logger->flush();
}

} // namespace

int main()
{
    SimHal hal {};
    dfw::runtime::Scheduler scheduler {hal.clock()};
    dfw::control::ControlLoop control_loop {hal};
    dfw::control::FlightModeManager flight_mode_manager {};
    dfw::control::ControlAllocator allocator {};
    dfw::comms::AiControlLink ai_control_link {};
    dfw::platform::IUartPort* telemetry_uart =
        hal.buses().create_uart_port(0, dfw::platform::UartConfig {});
    dfw::comms::TelemetryPublisher telemetry_publisher {*telemetry_uart};
    SimAppendStorageDevice log_device {};
    dfw::logging::SdLogStorage log_storage {log_device};
    dfw::logging::Logger<32> logger {log_storage};
    dfw::estimation::AttitudeEstimator attitude_estimator {};
    dfw::safety::SafetySupervisor safety_supervisor {};
    SimImuDevice imu_device {*hal.buses().create_spi_device(0, 0, {})};
    dfw::sensing::SensorManager sensor_manager {imu_device};
    ImuIrqContext imu_irq_context {&hal.clock(), &sensor_manager, &scheduler};
    dfw::sensing::ImuSample latest_imu_sample {};
    dfw::control::MotorOutputs latest_motor_outputs {};
    dfw::control::AllocatorStatus latest_allocator_status {};
    dfw::safety::SafetyOutput latest_safety_output {};
    dfw::control::PilotCommand pilot_command {};
    AppContext app_context {
        &hal,
        &scheduler,
        &control_loop,
        &flight_mode_manager,
        &allocator,
        &ai_control_link,
        &telemetry_publisher,
        &logger,
        &attitude_estimator,
        &safety_supervisor,
        &sensor_manager,
        &latest_imu_sample,
        &latest_motor_outputs,
        &latest_allocator_status,
        &latest_safety_output,
        &pilot_command,
    };

    control_loop.initialize();
    sensor_manager.initialize();
    log_storage.initialize();
    hal.sensor_interrupts().register_data_ready_line(0, imu_irq_callback, &imu_irq_context);

    (void) scheduler.add_event_task("imu_acquisition",
                                    dfw::runtime::Scheduler::PriorityClass::estimation_fast,
                                    150,
                                    &app_context,
                                    imu_acquisition_task);

    (void) scheduler.add_task("estimation_update",
                              dfw::runtime::Scheduler::PriorityClass::estimation_fast,
                              k_major_cycle_us,
                              150,
                              &app_context,
                              estimation_update_task);

    (void) scheduler.add_task("control_update",
                              dfw::runtime::Scheduler::PriorityClass::critical_fast,
                              k_major_cycle_us,
                              250,
                              &app_context,
                              control_update_task);

    (void) scheduler.add_task("output_update",
                              dfw::runtime::Scheduler::PriorityClass::critical_fast,
                              k_major_cycle_us,
                              100,
                              &app_context,
                              output_update_task);

    (void) scheduler.add_task("telemetry_publish",
                              dfw::runtime::Scheduler::PriorityClass::service_background,
                              20000,
                              1000,
                              &app_context,
                              telemetry_publish_task);

    (void) scheduler.add_task("log_flush",
                              dfw::runtime::Scheduler::PriorityClass::service_background,
                              50000,
                              1000,
                              &app_context,
                              log_flush_task);

    for (;;) {
        const dfw::common::TimestampUs cycle_start_us = hal.clock().now_us();
        imu_irq_callback(&imu_irq_context);
        scheduler.run_once();
        hal.board_control().service_watchdog();

        const dfw::common::TimestampUs cycle_end_us = hal.clock().now_us();
        if (cycle_end_us > cycle_start_us) {
            const dfw::common::DurationUs cycle_elapsed_us =
                static_cast<dfw::common::DurationUs>(cycle_end_us - cycle_start_us);
            if (cycle_elapsed_us < k_major_cycle_us) {
                hal.clock().sleep_us(k_major_cycle_us - cycle_elapsed_us);
            }
        }
    }
}
