# DroneFirmware Architecture Notes

## 1. ArduPilot-inspired insights, extracted as concepts only

The following items summarize reusable architectural ideas without reproducing code,
class designs, or repository organization.

### 1.1 Modular decomposition

- Separate platform-independent flight logic from hardware-specific drivers
- Group responsibilities by subsystem boundaries such as sensors, estimation, control, communications, and vehicle behavior
- Isolate optional capabilities behind interfaces and feature gates so the core runtime can remain lightweight
- Reuse shared service layers across multiple vehicle types instead of duplicating control infrastructure

### 1.2 Deterministic task scheduling

- Run the firmware as a set of periodic jobs with well-defined rates and budgets
- Give high-rate stabilization and actuator work precedence over low-rate housekeeping
- Keep loop timing explicit so overloads can be detected, measured, and mitigated
- Centralize execution policy rather than letting each subsystem spawn its own unmanaged loop

### 1.3 Sensor handling and data flow

- Abstract physical devices behind consistent interfaces for initialization, health reporting, sampling, and calibration
- Collect device data into a manager that can coordinate update timing, redundancy, and fault isolation
- Keep raw acquisition, conditioning, estimation, and control as separate stages in the pipeline
- Support degraded operation when a subset of sensors becomes unavailable

### 1.4 Control loop structure

- Use layered control where outer loops generate targets for inner loops
- Keep vehicle-specific motion logic separate from generic timing and I/O services
- Make arming, mode transitions, and actuator authority explicit to avoid hidden state changes
- Maintain a clear handoff between navigation intent, stabilization, and output mixing

### 1.5 Communication patterns

- Separate transport concerns from message semantics
- Support telemetry, command handling, parameter exchange, and health/status publication through a unified communications service
- Decouple internal control/state representations from externally visible protocols
- Permit multiple links with different reliability and bandwidth characteristics

### 1.6 Parameter and configuration management

- Store runtime-tunable settings in a central registry with metadata, validation, and persistence hooks
- Use parameters as a configuration interface rather than embedding constants throughout modules
- Distinguish operational tuning from compile-time feature selection
- Treat parameter changes as events that can trigger subsystem reconfiguration or safety checks

## 2. Proposed DroneFirmware architecture

DroneFirmware is organized around a runtime kernel plus domain modules. The design favors
stable interfaces and data contracts over inheritance-heavy frameworks.

### 2.1 Core modules

#### Runtime Kernel

- Starts the platform, owns the scheduler, and orchestrates startup phases
- Provides monotonic time, execution budgets, and lifecycle transitions
- Hosts service registration for modules that participate in the control pipeline

#### Platform Adaptation Layer

- Exposes board capabilities through small interfaces: clocks, buses, storage, GPIO, PWM, DMA, and watchdog
- Prevents control logic from depending on STM32-specific APIs
- Enables per-board implementations without touching higher layers

#### Sensor Domain

- Owns device drivers, sampling policies, health state, calibration application, and time alignment
- Publishes normalized measurements into a shared data model
- Shields estimators and controllers from bus- and chip-level details

#### State Estimation

- Fuses normalized measurements into attitude, position, velocity, and health confidence outputs
- Defines estimator products as contracts consumed by navigation and control
- Can swap implementations later without rewriting downstream modules

#### Mission and Guidance

- Converts operator commands, route plans, and autonomy outputs into motion targets
- Handles mode logic for air, ground, and marine products through pluggable behavior policies
- Sits above stabilization and below mission/autonomy orchestration

#### Control System

- Executes cascaded loops and converts desired motion into normalized actuator demands
- Keeps control laws independent from the physical actuator topology
- Accepts estimator state and guidance targets on deterministic schedules

#### Output and Allocation

- Maps normalized control demands onto motors, servos, thrusters, or steering actuators
- Applies limits, saturation handling, dead zones, and output-safe states
- Encapsulates vehicle-specific mixing/allocation strategies

#### Communications Fabric

- Manages links, sessions, protocol adapters, and message routing
- Supports telemetry, commands, logs, remote tuning, and integration with Drone Factory
- Allows external protocols to evolve without leaking into the core control path

#### Parameter and Configuration Service

- Registers parameters with metadata, defaults, ranges, mutability rules, and persistence policy
- Validates writes and emits change notifications
- Serves as the contract boundary between configuration tools and runtime modules

#### Safety Supervisor

- Monitors system health, arming state, watchdogs, link loss, estimator validity, battery constraints, and geofencing
- Owns failsafe policy selection and safe-output behavior
- Can preempt nominal control when operating conditions violate safety rules

### 2.2 Interfaces between modules

#### HAL to Runtime

- `IClock`: monotonic timestamps and sleep primitives
- `IBoardControl`: reset, boot reason, and watchdog control
- `IOutputDriver`: channel-level actuation primitives
- `IBusFactory`: access to SPI, I2C, UART, CAN, and storage services
- `IPwmBank`: grouped motor/servo outputs with protocol-aware configuration
- `ISensorInterruptRouter`: binding of IMU or sensor data-ready lines into the event system

#### Sensor Domain to Estimation

- `MeasurementFrame`: timestamped, normalized sensor samples with quality flags
- `SensorSnapshot`: coherent set of latest measurements for fusion ticks

#### Estimation to Guidance and Control

- `StateVector`: attitude, rates, position, velocity, and validity fields
- `NavigationStatus`: estimator confidence and aiding state

#### Guidance to Control

- `MotionTarget`: desired attitude, rates, heading, speed, climb, or track depending on vehicle type
- `ModeIntent`: desired control authority and mission context

#### Control to Output

- `ControlDemand`: normalized roll/pitch/yaw/thrust or steering/throttle commands
- `AllocationResult`: actuator commands, saturation flags, and clipping diagnostics

#### Parameters to All Modules

- `IParameterStore`: get, set, persist, and subscribe to changes
- `ParameterChangeEvent`: key, previous value, new value, and source

#### Safety across the system

- `HealthReport`: module status, severity, and recommended action
- `SafetyCommand`: arm inhibit, mode downgrade, output hold, controlled stop, or reboot request

## 2.3 Real-time execution model for STM32-class controllers

DroneFirmware should default to a cooperative, time-triggered executive rather than depending on a full RTOS.
That keeps timing behavior explicit, reduces integration complexity on smaller MCUs, and makes certification-style
reasoning easier. An RTOS can be introduced later for high-end boards if profiling shows clear benefit, but the
baseline architecture should not require one.

### Execution strategy

- Use a hardware timer as the system timebase with microsecond-resolution monotonic time
- Run interrupts only for bounded, latency-sensitive events such as timer compare, DMA completion, and sensor data-ready notification
- Keep heavy work out of ISRs; interrupts should capture timestamps, move bytes into buffers, set event flags, and return
- Execute all substantive logic in the main scheduler context where priorities, budgets, and ordering are deterministic

### Task prioritization model

Define four scheduler classes with fixed priority ordering:

1. `critical_fast`
   - Stabilization, rate control, output update, watchdog servicing, and safety interlocks
   - Must always finish within the smallest control period
2. `estimation_fast`
   - IMU preprocessing, state propagation, attitude update, and input freshness checks
   - Runs immediately before or alongside the inner control loop
3. `mission_normal`
   - Navigation, mode logic, parameter update application, command handling
   - Important but allowed to miss an occasional frame without destabilizing the vehicle
4. `service_background`
   - Telemetry packaging, logging, diagnostics, storage flush, low-rate health reporting
   - Can be rate-limited or skipped under CPU pressure

This gives predictable degradation: background work drops first, mission work next, while estimation and stabilization retain timing.
The intended execution pipeline is explicit:

1. IMU acquisition
2. Estimation update
3. Control update
4. Output update
5. Telemetry and logging

### Deterministic timing rules

- Each task has a fixed nominal period, a worst-case execution budget, and a next-release timestamp
- Scheduler order is static within each priority class to avoid run-to-run jitter
- All periodic jobs should be release-based, not delay-based, so schedule drift is bounded
- Overrun accounting should record deadline miss count, execution time high-water mark, and consecutive misses
- Scheduler observability should also track:
  - skipped periodic releases
  - lower-priority executions denied because there was not enough slack before the next higher-priority release
- If CPU load rises, the scheduler should skip or decimate lower-priority service tasks rather than slipping the fast loop
- Control and estimation paths should use timestamped data snapshots so logic remains deterministic even when sample arrival jitter exists
- Typical starter budgets in this firmware are:
  - `imu_acquisition`: `150 us`
  - `estimation_update`: `150 us`
  - `control_update`: `250 us`
  - `output_update`: `100 us`
  - `telemetry_publish`: `1000 us`
  - `log_flush`: `1000 us`
- If a periodic task is already later than its release time plus budget, the scheduler should skip that release instead of executing stale work

### Fast and slow loop separation

The system should explicitly split high-rate stabilization from slower vehicle behavior:

- Fast loop:
  - Runs at the control base rate
  - Consumes the latest inertial state
  - Produces actuator demands with minimal branching and no dynamic allocation
- Medium loop:
  - Handles navigation updates, mode transitions, outer-loop target generation, and input interpretation
- Slow loop:
  - Handles telemetry, parameter persistence, storage maintenance, diagnostics, and non-critical integrations

This separation prevents communications or storage traffic from perturbing stabilization timing.

### Interrupt versus main loop responsibilities

Interrupt context should own:

- Timer tick and capture events
- DMA completion for SPI/UART/CAN transfers
- GPIO data-ready edges from IMUs or other sensors
- Emergency hardware fault latching if a board requires immediate shutdown signaling

Main loop context should own:

- Sensor frame parsing and validation
- State estimation updates
- Control law execution
- Mission logic and autonomy
- Protocol parsing above byte/frame buffering
- Parameter validation and persistence

The practical rule is simple: ISRs announce work; the scheduler performs work.

### Example timing bands

Typical starting frequencies for an STM32 flight controller:

| Function | Target Rate | Notes |
|---|---:|---|
| IMU data-ready interrupt | 1-8 kHz | Hardware-driven, timestamp only plus DMA/buffer handoff |
| IMU processing / estimator propagation | 1 kHz | Fast attitude and rate state update |
| Inner attitude-rate control loop | 500-1,000 Hz | Main stabilization path |
| Motor/actuator output refresh | 400-1,000 Hz | Depends on protocol and actuator type |
| RC / pilot input update | 50-200 Hz | Link-dependent |
| Navigation / outer-loop guidance | 50-100 Hz | Position, velocity, track, waypoint logic |
| Barometer / magnetometer fusion | 50-100 Hz | Sensor-dependent |
| GNSS handling | 5-20 Hz | Receiver-dependent |
| Telemetry uplink/downlink | 10-50 Hz | Split by message importance |
| Logging / diagnostics | 10-25 Hz | Buffered and back-pressure aware |
| Parameter persistence / storage maintenance | 1-5 Hz | Deferred and wear-aware |

A practical baseline for a multirotor is:

- IMU interrupt at sensor rate, usually 4 kHz or 8 kHz
- Estimation at 1 kHz
- Inner rate loop at 1 kHz
- Attitude / target shaping at 250 Hz
- Navigation at 50 Hz
- Telemetry at 20 Hz
- Logging and diagnostics at 10 Hz

For ground and marine vehicles, the same framework can run with slower inner-loop rates while preserving the same priority model.

### Scalability for autonomy and multi-vehicle support

Future modules should join the scheduler through declared execution classes rather than ad hoc threads.

- Autonomy planners should run in `mission_normal` or a new low-rate `planning` class, typically 5-20 Hz
- Multi-vehicle coordination should be event-fed and budget-capped so link bursts cannot starve stabilization
- High-cost modules should support incremental work slices, allowing the scheduler to spread computation across cycles
- CPU reservation should be explicit: for example, no more than 20 percent of frame time allocated to autonomy on a flight controller
- On larger boards, the same task model can be mapped onto RTOS threads later, but the logical execution contract remains unchanged

### Recommended scheduler behavior

The scheduler should operate as a fixed-priority cyclic executive:

1. Latch current time and pending interrupt-driven events
2. Run due `estimation_fast` tasks in static order
3. Run due `critical_fast` tasks in static order
4. Run due `mission_normal` tasks if slack remains
5. Run `service_background` work opportunistically
6. Service watchdog and idle until the next release boundary or interrupt

The scheduler should measure each callback runtime, retain the last and maximum observed execution time, and increment an overrun counter whenever a task exceeds its configured budget.
Those per-task timing counters and global scheduler skip/denial counters should be exposed through the scheduler API so they can later be surfaced by telemetry or logging without changing the execution model.
Because this is a cooperative executive, it cannot preempt a task that has already started; practical enforcement comes from fixed budgets, overrun detection, skipping stale periodic releases, and refusing to start lower-priority work when there is not enough slack before the next higher-priority release.

This model is simple enough for STM32 parts, deterministic under load, and extensible as the platform grows.

## 2.4 HAL design for STM32-based flight controllers

The HAL should expose capability-oriented interfaces rather than mirror STM32 registers or vendor HAL calls.
Board code owns clocks, pins, DMA channels, and interrupt vectors. Flight logic sees only stable service contracts.

### HAL design goals

- Hide MCU-family details such as STM32 timer instances, SPI peripheral numbering, and DMA stream routing
- Make buses shareable across multiple sensors while keeping chip-select and address ownership explicit
- Support DMA-backed drivers where available without requiring DMA semantics in high-level modules
- Keep latency-sensitive input and output primitives available without allowing application code to touch registers
- Allow a SITL or host implementation to satisfy the same interfaces for testing

### Layering

Use three HAL layers:

1. `Platform contracts`
   - Pure interfaces used by the rest of the firmware
2. `Board support package`
   - Pin map, bus instantiation, timer allocation, interrupt wiring, clock setup
3. `Peripheral drivers`
   - STM32-specific implementations for SPI, I2C, UART, PWM, watchdog, and GPIO event routing

Only layer 1 is visible to control, sensing, and communications modules.

### Core HAL interfaces

#### Time and board control

- `IClock`
  - `now_us()`
  - `sleep_us()`
- `IBoardControl`
  - watchdog service
  - controlled reset
  - later extension for boot reason and brownout reporting

#### SPI

Use SPI for IMUs, fast barometers, FRAM, and some magnetometers.

- `ISpiDevice`
  - bound to a bus plus chip-select
  - supports full-duplex `transfer()`
  - device configuration includes clock rate, mode, and DMA preference
- `IBusFactory::create_spi_device()`
  - board support code decides which hardware SPI peripheral backs each logical bus

Design notes:

- One `ISpiDevice` instance should represent one physical peripheral on one bus, not the shared bus itself
- Arbitration belongs below the interface, so sensor drivers do not manage chip-select timing directly
- DMA usage should remain an implementation detail unless a module explicitly requests non-blocking transfer later

#### I2C

Use I2C for magnetometers, pressure sensors, EEPROMs, and low-rate peripheral expanders.

- `II2cDevice`
  - bound to a bus plus slave address
  - supports `write()` and combined `write_read()` for register access patterns
- `IBusFactory::create_i2c_device()`
  - uses logical bus identifiers so board variants can remap peripherals without changing drivers

Design notes:

- Bus recovery should be owned by the STM32 driver layer, not sensor code
- Timeout and retry policy should be centralized because stuck-bus handling is safety-relevant

#### UART

Use UART for telemetry radios, GNSS, RC receivers, companion computers, and debug consoles.

- `IUartPort`
  - `write()`, `read()`, `available()`
  - configured by baud rate, parity, stop bits, and DMA preference
- `IBusFactory::create_uart_port()`
  - returns an abstract port independent of USART instance numbers

Design notes:

- RX should be interrupt- or DMA-backed into a ring buffer
- TX `IUartPort::write()` should be a bounded enqueue-or-fail call, not a blocking full-transfer primitive
- A successful TX call should mean "accepted by the driver" through a FIFO or DMA-backed request path, not "all bytes fully shifted on the wire"
- If the TX path is full, the HAL should return `busy` immediately so low-rate tasks such as telemetry can skip the frame instead of stalling runtime
- Higher layers should parse framed protocols in the main loop, not in ISR context
- Ports should support different policy profiles later, such as low-latency RC versus bulk telemetry

#### PWM and actuator output

Motor and servo outputs need a stronger abstraction than a single write call.

- `IPwmBank`
  - `configure_channel()`
  - `write_normalized()`
  - `disarm_all()`
  - `channel_count()`
- `PwmChannelConfig`
  - update rate
  - min/max pulse range
  - protocol type such as analog PWM, OneShot, or DShot

Design notes:

- Output timing should be owned by timer/DMA configuration inside the board layer
- Mixing and allocation stay above the HAL; the HAL only drives already-computed commands
- Safety supervisor should be able to force `disarm_all()` regardless of vehicle type

#### Sensor interrupt routing

Sensor drivers often need deterministic wakeup from IMU data-ready pins.

- `ISensorInterruptRouter`
  - register callback against a logical external interrupt line
  - ISR sets event flags or timestamps; the scheduler consumes them later

This interface keeps EXTI details out of device drivers while preserving low-latency sampling.

### STM32 implementation guidance

- Use one high-resolution timer for the scheduler timebase
- Prefer DMA for SPI IMU reads, UART telemetry bursts, and high-rate DShot or timer updates where supported
- Keep peripheral IRQ handlers tiny: acknowledge hardware, stamp time, move data to a buffer, signal the scheduler
- Model all bus and output instances with logical identifiers so `board_a` and `board_b` can expose the same firmware-facing names
- Centralize pin assignments and alternate-function setup in board support files, not in drivers

### Sensor integration pattern

A typical STM32 sensor path should work like this:

1. Board support creates an `ISpiDevice` for the IMU and registers its data-ready interrupt line
2. ISR timestamps the edge and marks the IMU sample event pending
3. Scheduler runs the sensor acquisition task in `estimation_fast`
4. Sensor driver performs the SPI transfer through `ISpiDevice`
5. Sensor manager validates and publishes a normalized `MeasurementFrame`

This keeps the latency-critical trigger path short while leaving parsing and validation in deterministic main-loop code.

### Recommended file ownership

- `src/platform/Hal.hpp`
  - platform contracts only
- `boards/<board-name>/`
  - clocks, pin map, peripheral ownership, concrete HAL assembly
- `src/platform/stm32/`
  - reusable STM32 peripheral implementations shared across boards

That split keeps board variation from leaking into application code while still allowing optimized STM32 drivers underneath.

## 2.5 Scheduler and HAL integration

The scheduler should treat the HAL as the only path to hardware-facing work. Tasks do not manipulate buses,
timers, or interrupt lines directly. They consume HAL services, produce normalized data, and hand results to the
next stage of the pipeline.

### 1. Task interaction with HAL interfaces

#### Sensor read cycles

- High-rate sensors such as IMUs are usually connected through `ISpiDevice`
- Lower-rate sensors such as magnetometers or barometers may use `II2cDevice`
- GNSS and some receivers use `IUartPort`
- A scheduled sensor task owns the actual bus transaction and sensor frame parsing
- Sensor tasks publish normalized `MeasurementFrame` objects into the sensing domain after validating transport status and timestamps

Typical mapping:

- `imu_acquisition` task
  - triggered by IMU data-ready event
  - calls `ISpiDevice::transfer()`
  - updates the latest inertial sample buffer
- `gps_update` task
  - periodic, for example 10 Hz
  - checks `IUartPort::available()` and drains bytes with `read()`
  - parser updates navigation input state
- `baro_update` task
  - periodic, for example 50 Hz
  - calls `II2cDevice::write_read()`
  - publishes pressure and temperature sample

#### Actuator updates

- Control and allocation tasks compute normalized outputs in the range expected by the output layer
- The final output task uses `IPwmBank::write_normalized()` or `IOutputDriver::write_channel()` depending on whether the system is addressing grouped motor outputs or generic channels
- Output protocol details such as PWM pulse timing or DShot waveform generation remain inside the HAL implementation

Typical mapping:

- `output_update` task
  - periodic in `critical_fast`
  - consumes the most recent `ControlDemand` or `AllocationResult`
  - writes each active actuator channel through `IPwmBank`

#### Communication

- UART interrupt or DMA fills a receive buffer in the HAL implementation
- A scheduled communications task drains bytes from `IUartPort`, parses frames, and publishes commands or telemetry state
- Transmit is also initiated by scheduled tasks so message rate and bandwidth usage remain budgeted

Typical mapping:

- `telemetry_rx` task
  - periodic in `mission_normal`
  - reads available bytes from `IUartPort`
  - decodes commands and parameter requests
- `telemetry_tx` task
  - periodic in `service_background`
  - serializes state and sends through `IUartPort::write()`

### 2. Timing model

#### Interrupts versus scheduled tasks

Interrupts should only handle:

- sensor data-ready edge capture through `ISensorInterruptRouter`
- SPI/UART/I2C DMA completion acknowledgment inside the HAL
- timer compare or output hardware latch events
- minimal byte capture for non-DMA UART receive paths

Scheduled tasks should handle:

- full sensor reads and sample validation
- estimator updates
- control loop computation
- actuator command publication
- protocol parsing and telemetry packaging

#### Sensor data-ready propagation

The propagation path should be:

1. Board support registers a callback with `ISensorInterruptRouter`
2. The IRQ callback stores a timestamp from `IClock` or a hardware timer capture
3. The callback marks an event for the scheduler, for example `scheduler.notify_event("imu_acquisition")`
4. The next scheduler cycle runs the `imu_acquisition` event task in `estimation_fast`
5. The sensor task performs the HAL bus transfer and publishes the resulting sample

This gives low interrupt latency without moving parsing or control math into IRQ context.

#### Deterministic timing guarantees

Determinism comes from four rules:

- fixed priority classes and static task order inside each class
- release-based task timing using `next_release_us`
- event-driven tasks limited to short acquisition or dispatch work
- no blocking waits in fast tasks beyond bounded HAL transfers already budgeted for that period

A task is deterministic if its input snapshot, execution budget, and release source are all explicit.

### 3. Data flow

The nominal high-rate control path should be:

1. Sensor hardware asserts data-ready
2. HAL interrupt router records the event
3. Scheduler runs sensor acquisition task
4. Sensor task reads hardware through `ISpiDevice` or `II2cDevice`
5. Sensor manager normalizes and timestamps the sample as `MeasurementFrame`
6. Estimator task converts measurement sets into `StateVector`
7. Control task combines `StateVector` and `MotionTarget` into `ControlDemand`
8. Output task converts demand into channel writes through `IPwmBank`

Communication follows the same pattern:

1. UART hardware receives bytes
2. HAL buffers bytes
3. Scheduled comms task drains and parses them
4. Resulting commands update guidance, parameters, or mission state

### 4. Threading and execution model

DroneFirmware should use a single cooperative main loop plus interrupts as the default execution model.

Justification:

- easier to reason about worst-case latency on MCU-class targets
- avoids priority inversion and stack growth from multiple threads
- matches the fixed-rate control problem better than general-purpose preemption
- makes HAL ownership simpler because each peripheral is serviced by one deterministic software path

An RTOS-style mapping can remain a future implementation option for high-end boards, but the architectural contract should still look like one logical scheduler with interrupt-fed events. In other words, RTOS is an implementation detail, not the programming model.

### 5. Minimal execution example

Minimal high-rate flow using the current interfaces:

1. Startup:
   - create `ISpiDevice` for the IMU through `IHal::buses()`
   - configure motor outputs through `IHal::pwm_bank()`
   - register IMU data-ready callback through `IHal::sensor_interrupts()`
2. Interrupt:
   - IMU line toggles
   - callback marks `imu_acquisition` event
3. Scheduled acquisition:
   - `imu_acquisition` runs in `estimation_fast`
   - reads one IMU frame using `ISpiDevice::transfer()`
   - stores normalized inertial sample
4. Scheduled control:
   - `control_update` runs periodically in `critical_fast`
   - estimator consumes latest inertial sample and updates `StateVector`
   - controller computes `ControlDemand`
5. Scheduled output:
   - `output_update` runs in `critical_fast` after control
   - writes normalized motor commands via `IPwmBank::write_normalized()`

Example rates:

- IMU interrupt: 4-8 kHz
- `imu_acquisition`: event-driven, typically one run per IMU sample
- `control_update`: 1 kHz
- `output_update`: 1 kHz or hardware-synchronized at the motor protocol rate
- UART telemetry tasks: 20-50 Hz for transmit, 50-200 Hz polling/drain for receive depending on link type

### Minimal execution pseudocode

```cpp
// Startup wiring
auto* imu = hal.buses().create_spi_device(0, 0, imu_spi_config);
hal.sensor_interrupts().register_data_ready_line(0, imu_irq_callback, &scheduler);

scheduler.add_event_task("imu_acquisition",
                         Scheduler::PriorityClass::estimation_fast,
                         150,
                         [&](TimestampUs now_us) {
                             read_imu_sample(*imu, now_us);
                         });

scheduler.add_task("control_update",
                   Scheduler::PriorityClass::critical_fast,
                   1000,
                   250,
                   [&](TimestampUs now_us) {
                       estimate_state(now_us);
                       run_controller(now_us);
                   });

scheduler.add_task("output_update",
                   Scheduler::PriorityClass::critical_fast,
                   1000,
                   100,
                   [&](TimestampUs now_us) {
                       publish_outputs(now_us, hal.pwm_bank());
                   });

void imu_irq_callback(void* context)
{
    auto* scheduler = static_cast<Scheduler*>(context);
    scheduler->notify_event("imu_acquisition");
}
```

This model stays hardware-agnostic because tasks know only the HAL contracts and scheduler semantics, not STM32 registers or vendor APIs.

## 2.6 Minimal SensorManager and IMU data contract

The sensing side should stay small and explicit. A minimal `SensorManager` is enough if it owns device handles,
triggers acquisition, and publishes stable samples to the rest of the system.

### 1. SensorManager responsibilities

`SensorManager` should own:

- sensor device handles created via the HAL, for example an IMU `ISpiDevice` wrapped by a small `IImuDevice`
- acquisition entry points used by scheduled tasks
- the latest stable sensor sample buffer
- trigger state received from sensor data-ready callbacks

`SensorManager` should not own:

- estimator logic
- control laws
- mission behavior
- direct interrupt handling beyond recording a trigger event and timestamp

In the minimal design:

- IMU interrupt records that a new sample is ready
- `imu_acquisition` task performs the actual bus read
- `control_update` reads the latest published IMU sample

### 2. Data model

The IMU contract should include both physical values and timing metadata:

- `sample_time_us`
  - timestamp associated with sensor readiness
- `acquisition_time_us`
  - time when the scheduled task read the sample
- `transport_latency_us`
  - difference between trigger and acquisition
- `sequence`
  - monotonically increasing sample number
- `accel_mps2[3]`
  - acceleration in body axes
- `gyro_rad_s[3]`
  - angular rate in body axes
- `temperature_c`
  - optional thermal context for compensation
- `valid`
  - indicates a complete, publishable sample

This is enough for a deterministic handoff to the estimator or directly to a placeholder control loop.

### 3. Buffering strategy

Use a double buffer.

Why not a single buffer:

- a reader could observe partially updated fields if the writer is interrupted or if the design later grows toward multi-context access

Why not a ring buffer in the minimal design:

- extra indexing and overflow policy add complexity that the control loop does not need when it only consumes the latest stable sample

Double-buffer approach:

- acquisition writes a full sample into the inactive slot
- once complete, it flips the published index
- readers always copy from the last published slot

This gives a clear handoff with minimal state.

### Avoiding race conditions

Two paths exist:

- ISR path
  - only records `imu_trigger_pending` and `imu_trigger_time_us`
- scheduled acquisition path
  - reads the sensor and publishes a complete `ImuSample`

Because ISR code never writes into the sample buffer, the main race is between publishing and reading. The double buffer solves that by ensuring the reader only sees fully published samples.

### 4. Data flow and handoff semantics

The handoff contract should be:

1. IMU data-ready interrupt occurs
2. ISR records trigger timestamp and marks acquisition pending
3. Scheduler runs `imu_acquisition`
4. `SensorManager` reads hardware through the IMU device
5. `SensorManager` writes the sample into the inactive buffer slot
6. `SensorManager` flips the published slot index
7. `control_update` copies the latest published sample

Semantics:

- writer publishes only complete samples
- reader gets the newest complete sample available at read time
- missed IMU events may collapse into the latest sample, which is acceptable for a minimal latest-sample contract

### 5. Scheduler integration

- `imu_acquisition`
  - event-driven
  - priority class `estimation_fast`
  - runs when the IMU data-ready callback marks work pending
- `control_update`
  - periodic
  - priority class `critical_fast`
  - runs at the control rate, for example 1 kHz

Validity guarantee:

- control only runs the IMU-dependent path if `SensorManager::get_latest_imu_sample()` returns true
- once at least one valid sample has been published, every subsequent control tick uses the latest stable sample
- if acquisition fails or data is absent, control can hold outputs, reuse the previous state estimate, or invoke a safety response

### 6. Minimal implementation shape

- `SensorManager.hpp`
  - owns trigger state and double buffer
- `ImuSample`
  - timing-aware sample contract
- `acquire_imu()`
  - scheduled writer path
- `get_latest_imu_sample()`
  - control-loop reader path

This is the right level of complexity for STM32: deterministic, cheap, and easy to inspect.

## 2.7 Timestamp propagation from HAL to SensorManager

Timestamp handling should use one monotonic timebase owned by the HAL and shared across interrupts,
scheduled acquisition, and control. The goal is not absolute wall-clock time. The goal is stable,
comparable microsecond timestamps for control timing.

### 1. Time source

The time source should be:

- a free-running hardware timer or microsecond counter maintained by the board layer
- monotonic for the duration of normal operation
- readable from both task context and ISR context

Recommended characteristics:

- unit: microseconds
- resolution: 1 microsecond
- precision goal: good enough that trigger-to-acquisition latency and sample-to-sample `dt` are meaningful for 1 kHz control
- representation: `TimestampUs` as a 64-bit unsigned value to avoid short wrap intervals at the architecture level

The board implementation may use a 32-bit timer internally and extend it in software if needed, but that should stay hidden behind the HAL.

### 2. HAL responsibility

The HAL clock interface should provide two reads on the same timebase:

- `now_us()`
  - used in scheduled task context
- `now_us_isr()`
  - used in interrupt context

This keeps timestamp capture explicit and prevents application code from guessing whether a time read is safe inside an ISR.

The HAL is responsible for:

- configuring the underlying timer or counter
- guaranteeing that `now_us()` and `now_us_isr()` are consistent with each other
- keeping the read path lightweight enough for high-rate IMU interrupts

### 3. ISR behavior

When the IMU data-ready interrupt fires:

1. ISR reads `clock.now_us_isr()`
2. ISR passes that timestamp to `SensorManager::notify_imu_trigger()`
3. ISR marks `imu_acquisition` pending in the scheduler
4. ISR returns immediately

Important rule:

- the timestamp captured in the ISR is the authoritative sample-ready time

This avoids replacing a precise hardware-edge time with a delayed scheduler timestamp.

### 4. SensorManager propagation

`SensorManager` should preserve both timing points:

- `sample_time_us`
  - timestamp captured in the data-ready ISR
- `acquisition_time_us`
  - timestamp when the scheduled acquisition task performs the bus read

From those, it derives:

- `transport_latency_us`
  - `acquisition_time_us - sample_time_us`
- `sample_period_us`
  - difference from the previously published `sample_time_us`

That gives downstream logic both the physical sample timing and the software servicing delay.

### 5. Control loop timing

The control loop should compute `dt` from successive IMU sample timestamps, not from when the control task happened to run.

Recommended logic:

- if a previous sample exists, compute `dt = current.sample_time_us - previous.sample_time_us`
- if this is the first sample, use zero or a conservative default until a second sample arrives
- keep `dt` explicit inside the control loop state

This makes timing deterministic because:

- sensor cadence is tracked from the actual sensor trigger path
- scheduler jitter does not directly distort the control integration step

### 6. Minimal implementation shape

Minimal code responsibilities:

- `TimestampUs`
  - defined in shared time types
- `IClock`
  - provides `now_us()` and `now_us_isr()`
- ISR callback
  - captures timestamp and notifies both `SensorManager` and `Scheduler`
- `SensorManager`
  - stores trigger timestamp and propagates it into `ImuSample`
- `ControlLoop`
  - computes `dt` from successive `sample_time_us` values

This is enough for STM32-class firmware without introducing OS services, dynamic allocation, or unnecessary timing frameworks.

## 2.8 IMU sample validity and freshness in the control path

The control path should reject bad IMU timing explicitly rather than assuming every published sample is safe to use.
This keeps the behavior deterministic and creates a clean hook for later safety escalation.

### 1. Validity rules for `ImuSample`

Minimal production-oriented checks:

- maximum sample age
  - reject the sample if `now_us - sample_time_us` exceeds a configured freshness limit
  - practical starter value: 5 ms
- acceptable sample period range
  - if `sample_period_us` is present, require it to stay within an expected band
  - practical starter band: 250 us to 4000 us
- acceptable transport latency range
  - reject if `transport_latency_us` is too large
  - practical starter value: 1 ms
- invalid timestamp conditions
  - `sample_time_us == 0`
  - `acquisition_time_us == 0`
  - `acquisition_time_us < sample_time_us`
  - `now_us < sample_time_us`

These checks are intentionally small and cheap. They catch stale, inconsistent, and badly delayed samples without adding estimator complexity.

### 2. Control loop behavior

When the IMU sample is invalid:

- do not compute a new control update
- clear or hold the internal control demand in a known safe state
- mark the control result as not safe for output publication

When the sample is valid:

- compute `dt` from the sample timestamps
- update `ControlDemand`
- mark the control result as output-valid

This lets `output_update` refuse to drive actuators from bad sensor timing.

### 3. Lightweight status type

Use a small enum such as:

- `ok`
- `no_data`
- `invalid_timestamp`
- `stale_sample`
- `invalid_period`
- `excessive_latency`

This is enough for:

- local control decisions
- logging or diagnostics later
- future `SafetySupervisor` integration without redesigning the interface

### 4. Integration rules

- `SensorManager` continues publishing the latest complete sample
- `ControlLoop` validates the sample before using it
- `ControlLoop` exposes a small `ControlUpdateResult`
- `output_update` checks that result before sending commands to `IPwmBank`

Minimal output rule:

- if the control result is invalid, do not publish actuator commands
- instead move outputs to a safe state, for example `disarm_all()`

### 5. Minimal implementation shape

The starter implementation should contain:

- `ImuSampleStatus`
  - stored on the sample contract
- `validate_imu_sample()`
  - small helper in the control path
- `ControlUpdateResult`
  - reports whether outputs are valid plus the sample status
- guarded `output_update`
  - publishes only when control output is valid

This keeps the timing and safety contract explicit without over-engineering the sensing stack.

## 2.9 Minimal rate control loop using IMU gyro data

The first useful closed-loop controller in the stack should be a body-rate controller. Its job is simple:
drive measured roll, pitch, and yaw rates toward commanded angular-rate setpoints.

### 1. Control objective

The objective is angular-rate stabilization for:

- roll rate
- pitch rate
- yaw rate

This is the inner loop. It should run at the fast control rate and produce normalized actuator demands that a later mixer or allocator can use directly.

### 2. Inputs

The controller needs:

- measured angular rates from `ImuSample::gyro_rad_s`
- desired rate setpoints from `MotionTarget`
- `dt` derived from successive IMU timestamps

Minimal mapping:

- `gyro_rad_s[0]` -> roll axis
- `gyro_rad_s[1]` -> pitch axis
- `gyro_rad_s[2]` -> yaw axis

### 3. Control structure

Use one small PID controller per axis.

- roll: PID
- pitch: PID
- yaw: PI or PID with a smaller derivative term

Each axis keeps:

- proportional gain
- integral gain
- derivative gain
- integrator clamp
- output clamp
- a small derivative filter coefficient

The production-oriented minimum should also:

- stop integrator growth when the controller is already saturated in the same direction as the error
- prefer derivative on measured rate instead of derivative on error to reduce setpoint-kick sensitivity
- apply a small first-order filter to the derivative term so gyro noise does not dominate tuning

This is enough to build a more stable starter loop without introducing heavy filtering or allocation complexity.

### 4. Timing

`dt` should come from IMU sample timing, not scheduler timing.

Rules:

- compute `dt` from successive `sample_time_us` values
- if timing is invalid, reject the sample
- clamp `dt` to a reasonable range before using it in PID integration and derivative calculations

This protects the controller from occasional jitter while preserving deterministic behavior.

### 5. Output

The rate loop should output normalized axis commands:

- roll in `[-1, 1]`
- pitch in `[-1, 1]`
- yaw in `[-1, 1]`
- thrust passed through separately

These are not yet motor commands. They are normalized control demands for the output/allocation layer.

### 6. Safety

- if the IMU sample is invalid, do not run the rate controller
- reset controller integrators when invalid data is detected
- keep `ControlUpdateResult::output_valid` false so `output_update` will not publish actuator commands

This makes the rate controller compatible with the existing output validity gate.

### 7. Minimal implementation shape

The starter implementation should include:

- `RateController`
  - owns per-axis PID state
- `RateController::update()`
  - takes setpoint, measured rate, and `dt`
- `ControlLoop`
  - holds one controller for each axis
  - validates the sample, computes `dt`, updates controllers, and publishes `ControlDemand`

This is the minimum practical rate loop for STM32-class firmware: small, deterministic, and easy to reason about.

## 2.10 Minimal quad-X motor mixer

The next step after rate control is a small actuator allocation layer that converts normalized
roll, pitch, yaw, and thrust demands into four motor outputs for a quadcopter in X configuration.

### 1. Inputs

The mixer accepts:

- normalized roll command in `[-1, 1]`
- normalized pitch command in `[-1, 1]`
- normalized yaw command in `[-1, 1]`
- normalized thrust in `[0, 1]`

These inputs come directly from `ControlDemand`.

### 2. Output

The mixer produces four motor commands:

- `m0`
- `m1`
- `m2`
- `m3`

Each output is normalized to `[0, 1]`.

### 3. Mixing logic

Use a fixed quad-X layout:

- `m0`: front-right
- `m1`: rear-right
- `m2`: rear-left
- `m3`: front-left

Minimal mix:

- `m0 = thrust - roll - pitch + yaw`
- `m1 = thrust - roll + pitch - yaw`
- `m2 = thrust + roll + pitch + yaw`
- `m3 = thrust + roll - pitch - yaw`

This preserves symmetry and gives the expected sign pattern for an X-frame starter implementation.

### 4. Saturation handling and constraints

The problem with simple per-motor clamping is that once one or more motors hit `0` or `1`,
the relative motor differences no longer match the commanded roll, pitch, and yaw balance.
That distorts control authority and can bias the vehicle.

A minimal improvement is:

1. compute the four raw control contributions around thrust
2. detect the largest absolute control term
3. compute how much symmetric headroom exists above and below thrust
4. rescale all four control contributions by the same factor if needed
5. add the rescaled terms back to thrust and clamp once at the end

This preserves:

- symmetry between opposite motors
- relative differences between motors
- non-negative outputs
- deterministic, lightweight behavior

It does not fully solve saturation management, but it preserves control balance much better than independent clipping.

### 5. Integration

- `ControlLoop` produces `ControlDemand`
- `Mixer` converts that demand to four motor outputs
- `output_update` writes those outputs to the first four `IPwmBank` channels

If control output is invalid, the existing validity gate still prevents publication and sends outputs to `disarm_all()`.

### 6. Minimal implementation shape

The starter implementation should contain:

- `Mixer`
  - stateless quad-X mixer with proportional saturation handling
- `mix()`
  - converts `ControlDemand` to `MotorOutputs`
- `output_update`
  - writes the four outputs to `IPwmBank`

This is enough to express a usable quadcopter allocation path while keeping the code deterministic and easy to inspect.

## 2.11 Minimal attitude estimation using IMU data

The first attitude estimator should stay deliberately small: estimate roll and pitch from IMU data
with a complementary filter, and keep yaw as gyro-integrated only for now.

### 1. Goal

The goal is to estimate:

- roll angle
- pitch angle
- optional yaw angle from gyro integration only

The estimator should be lightweight, deterministic, and fast enough to run in the same high-rate loop as rate control.

### 2. Inputs

The estimator uses:

- gyro angular rates from `ImuSample::gyro_rad_s`
- accelerometer measurements from `ImuSample::accel_mps2`

Interpretation:

- gyro provides fast short-term rotational motion
- accelerometer provides a gravity reference for long-term roll and pitch correction

### 3. Method

Use a complementary filter:

1. integrate gyro rates over `dt`
2. compute roll and pitch from the accelerometer gravity vector
3. blend gyro-integrated attitude with accelerometer-derived attitude using a small correction gain

Minimal formulas:

- `roll += gyro_x * dt`
- `pitch += gyro_y * dt`
- `yaw += gyro_z * dt`
- `accel_roll = atan2(accel_y, accel_z)`
- `accel_pitch = atan2(-accel_x, sqrt(accel_y^2 + accel_z^2))`
- `estimated = (1 - alpha) * gyro_integrated + alpha * accel_angle`

This keeps the implementation simple and STM32-friendly.

### 4. Timing

- compute `dt` from IMU sample timestamps
- clamp `dt` to a reasonable range before integration
- run the estimator in the same fast loop as rate control, after a valid IMU sample is available

This keeps timing consistent with the IMU-driven sensor pipeline.

### 5. Output

The estimator should publish a small attitude state:

- `roll_rad`
- `pitch_rad`
- `yaw_rad`
- `dt_s`
- `valid`

That is enough for logging, future mode logic, and later outer-loop attitude control.

### 6. Integration

- `SensorManager` provides the latest `ImuSample`
- `AttitudeEstimator` updates from that sample
- `ControlLoop` can continue using gyro rate data directly for the inner loop
- later, an outer-loop attitude controller can consume `AttitudeState` and generate rate setpoints

This keeps the current rate loop unchanged while preparing the architecture for cascaded control.

### 7. Minimal implementation shape

The starter implementation should contain:

- `AttitudeEstimator`
  - owns current attitude state and previous timestamp
- `update()`
  - takes an `ImuSample`, computes `dt`, integrates gyro, and applies accelerometer correction
- `AttitudeState`
  - stores roll, pitch, yaw, and validity

This is the minimum practical attitude estimator before introducing more advanced observers or EKF-style fusion.

## 2.12 Minimal outer-loop attitude controller

With attitude estimation available, the next layer is a small outer-loop controller that converts
desired roll and pitch angles into rate targets for the existing inner rate loop.

### 1. Goal

The goal is to stabilize:

- roll attitude
- pitch attitude

Yaw remains a direct rate command for now. This keeps the control chain simple:

- outer loop controls angles
- inner loop controls angular rates

### 2. Inputs

The controller uses:

- desired roll angle
- desired pitch angle
- desired yaw rate
- desired thrust
- current `AttitudeState`

### 3. Output

The output is a small rate-target setpoint:

- `roll_rate_target_rad_s`
- `pitch_rate_target_rad_s`
- `yaw_rate_target_rad_s`
- `thrust_target`
- `valid`

This can feed the existing `ControlLoop` without changing the inner-loop design.

### 4. Control law

Use a simple proportional controller on angle error:

- `roll_rate_target = kp_roll * (roll_target - roll_estimate)`
- `pitch_rate_target = kp_pitch * (pitch_target - pitch_estimate)`

Then clamp each rate target to a safe limit.

Minimal starter behavior:

- roll and pitch use proportional-only control
- yaw is passed through as a direct rate command
- thrust is passed through unchanged

This is the simplest useful outer loop before adding integrators, feedforward, or coordinated attitude logic.

### 5. Safety

- if `AttitudeState` is invalid, do not generate valid rate targets
- return an invalid setpoint result
- allow the existing downstream control/output validity flow to suppress actuator output

This keeps the failure path explicit and deterministic.

### 6. Integration

The intended flow becomes:

1. `SensorManager` publishes `ImuSample`
2. `AttitudeEstimator` updates `AttitudeState`
3. `AttitudeController` converts desired angles to rate targets
4. `ControlLoop` tracks those rate targets using gyro data
5. `Mixer` allocates outputs to motors

This preserves the existing inner rate loop and cleanly establishes a cascaded architecture.

### 7. Minimal implementation shape

The starter implementation should contain:

- `AttitudeController`
  - holds angle-to-rate proportional gains and rate clamps
- `update()`
  - takes `AttitudeTarget` plus `AttitudeState`
  - returns a `RateTargetSetpoint`

This is the minimum practical outer-loop attitude stage for an STM32-friendly cascaded controller.

## 2.13 Minimal pilot command interface

The control stack also needs a small, source-agnostic command mapping stage that converts user inputs
into an `AttitudeTarget`. This layer should not depend on RC hardware, joystick APIs, or any transport details.

### 1. Inputs

Use normalized generic pilot commands:

- roll input in `[-1, 1]`
- pitch input in `[-1, 1]`
- yaw input in `[-1, 1]`
- throttle input in `[0, 1]`

These inputs can come from:

- RC receiver decoding
- joystick or GCS control
- software-generated commands

### 2. Mapping

Map normalized inputs as follows:

- roll input -> roll angle target
- pitch input -> pitch angle target
- yaw input -> yaw rate target
- throttle input -> thrust target

Minimal formulas:

- `roll_target_rad = roll_input * max_roll_angle_rad`
- `pitch_target_rad = pitch_input * max_pitch_angle_rad`
- `yaw_rate_target_rad_s = yaw_input * max_yaw_rate_rad_s`
- `thrust_target = throttle_input`

### 3. Limits

Starter limits:

- maximum roll angle: `+-30 deg`
- maximum pitch angle: `+-30 deg`
- maximum yaw rate: approximately `2.5 rad/s`
- throttle range: `[0, 1]`

These should be enforced in the mapper regardless of input source.

### 4. Output

The mapper produces an `AttitudeTarget`:

- `roll_target_rad`
- `pitch_target_rad`
- `yaw_rate_target_rad_s`
- `thrust_target`

This feeds directly into the outer-loop `AttitudeController`.

### 5. Safety

- clamp all normalized inputs before mapping
- clamp throttle into `[0, 1]`
- ensure resulting targets stay within configured bounds
- if the input is marked invalid, return a default safe target

This keeps the command interface deterministic and source-independent.

### 6. Minimal implementation shape

The starter implementation should contain:

- `PilotInput`
  - normalized command container
- `InputMapper`
  - owns command limits
- `map()`
  - converts `PilotInput` into `AttitudeTarget`

This is enough to establish a clean input path for RC, joystick, or software-command integration later.

## 2.14 Minimal SafetySupervisor and flight state machine

The control stack also needs a small supervisory layer that decides whether motor outputs are allowed at all.
This layer should remain separate from the control laws so arming and failsafe behavior stays explicit.

### 1. States

Use three states:

- `DISARMED`
- `ARMED`
- `FAILSAFE`

These are enough for a minimal flight-state model.

### 2. Inputs

The production-oriented minimum should consume one compact health snapshot:

- `command_valid`
- `arm_command`
- `disarm_command`
- `control_output_valid`
- `sensor_valid`
- `attitude_valid`
- `watchdog_healthy`
- `throttle_low`
- `flight_mode_valid`
- `now_us`

This keeps the decision boundary explicit while staying small enough for a single fast task update.
In the current demo wiring, the pilot command is refreshed every fast-loop cycle, so the timeout path is present but not intentionally exercised by the example source.

### 3. Behavior

Required behavior:

- only allow motor output in `ARMED`
- `disarm_command` always wins, regardless of current state
- allow arming only when command, mode, control, attitude, sensors, and watchdog are valid and throttle is low
- debounce in-flight faults for `100 ms` before transitioning from `ARMED` to `FAILSAFE`
- once in `FAILSAFE`, remain latched until an explicit disarm command
- block motor output in both `DISARMED` and `FAILSAFE`

### 4. Integration

The supervisor sits between control and output:

1. `ControlLoop` computes `ControlDemand`
2. `SafetySupervisor` evaluates current system health
3. if motor output is allowed, `Mixer` computes the final motor commands
4. otherwise, the output path calls `disarm_all()`

This gives one explicit gate for actuator publication.

### 5. Output

The supervisor should publish:

- current `FlightState`
- `allow_motor_output`
- `arming_allowed`
- `block_reason`

`block_reason` should stay lightweight and enum-based so it can feed telemetry, logging, and a future higher-level safety UI without string formatting in the flight path.

### 6. Minimal implementation shape

The starter implementation should contain:

- `FlightState`
  - `DISARMED`, `ARMED`, `FAILSAFE`
- `SafetyInput`
  - explicit command, health, timing, and throttle-low inputs
- `SafetyOutput`
  - state, publish permission, arming permission, and block reason
- `SafetySupervisor`
  - `update()` implementing the state transitions and fault debounce

This remains a deliberately small supervisor, but it is strong enough to support explicit arming policy, debounced failsafe entry, and latched recovery behavior without introducing an RTOS or a larger event framework.

## 2.15 Minimal flight mode system

The control stack also needs a small mode-selection layer that decides how pilot input is interpreted.
For a minimal quadrotor controller, two modes are enough:

- `STABILIZE`
- `ACRO`

### 1. Modes

- `STABILIZE`
  - pilot roll and pitch inputs represent desired attitude angles
  - the outer-loop `AttitudeController` converts those angles to rate setpoints
- `ACRO`
  - pilot roll, pitch, and yaw inputs map directly to body-rate targets
  - the outer attitude loop is bypassed

### 2. Inputs

The mode system uses:

- `PilotInput`
- selected `FlightMode`
- current `AttitudeState`

### 3. Behavior

Required behavior:

- in `STABILIZE`, map pilot input to `AttitudeTarget`, then use `AttitudeController`
- in `ACRO`, map pilot roll, pitch, and yaw inputs directly to rate targets
- in both modes, pass throttle through as thrust

This keeps the inner rate loop unchanged while allowing different pilot feel and control paths.

### 4. Integration

The intended path becomes:

1. pilot input enters the mode layer
2. `FlightModeManager` selects the control path
3. it produces a `RateTargetSetpoint`
4. `ControlLoop` tracks those rate targets

This places the mode system between input interpretation and the inner rate controller.

### 5. Output

The mode layer produces:

- `RateTargetSetpoint`

That keeps the downstream interfaces unchanged.

### 6. Minimal implementation shape

The starter implementation should contain:

- `FlightMode`
  - `STABILIZE`, `ACRO`
- `FlightModeManager`
  - `update()` method selecting the correct path
- `STABILIZE`
  - `PilotInput -> AttitudeTarget -> AttitudeController -> RateTargetSetpoint`
- `ACRO`
  - `PilotInput -> direct rate targets`

This is enough to establish simple switching logic without adding complex mode infrastructure.

## 2.16 Unified pilot command path

Pilot intent should flow through the system as one lightweight command object rather than splitting
mode selection, arm state, and stick input across separate code paths.

### 1. `PilotCommand`

Use a small structure containing:

- `PilotInput`
- selected `FlightMode`
- arm command
- disarm command
- valid flag

This keeps operator intent explicit and source-independent.

### 2. Integration

The unified command path should work like this:

1. a source creates one `PilotCommand`
2. `FlightModeManager` consumes that command and produces `RateTargetSetpoint`
3. `SafetySupervisor` consumes the same command for arm/disarm decisions
4. the main loop forwards that single object as the pilot-intent source

This removes hardcoded mode and arm state from downstream logic.

### 3. Behavior

Required behavior:

- mode selection is explicit in the command path
- arm and disarm are explicit in the command path
- invalid command produces safe behavior

Minimal safe behavior:

- invalid command -> invalid rate targets
- invalid command -> no arming
- invalid command -> actuator output remains blocked

### 4. Minimal implementation shape

The starter implementation should contain:

- `PilotCommand`
  - carries input, mode, arm/disarm, and validity
- updated `FlightModeManager`
  - consumes `PilotCommand`
- updated safety integration
  - arm/disarm comes from the same command object
- main loop
  - uses one command object as the source of pilot intent

This is enough to unify the pilot command path without adding transport-specific dependencies or complex command routing.

## 2.17 Minimal parameter system

The firmware also needs a small static parameter system so control gains, limits, and constants can be configured
without scattering hardcoded values throughout the modules.

### 1. Goals

The parameter system should:

- configure gains, limits, and constants
- stay deterministic and embedded-friendly
- avoid heap allocation and runtime registration complexity
- use direct indexed lookup from `ParameterId`

### 2. Requirements

Use:

- static parameter definitions
- unique parameter identifiers
- type-safe access for `float`, `int`, and `bool`
- explicit default, minimum, and maximum values per parameter

This is enough for an STM32-class baseline without adding reflection or dynamic schema tooling.

### 3. Structure

Minimal building blocks:

- `ParameterId`
  - unique enum identifier for each parameter
- `Parameter`
  - name, type, current value, default value, and min/max limits
- `ParameterRegistry`
  - statically allocated array indexed directly by `ParameterId`
  - typed `get_*()` and `set_*()` methods
  - range validation on writes
  - `reset_to_defaults()`

This keeps access simple and predictable.

### 4. Integration

Intended usage pattern:

- `RateController`
  - consumes gain parameters
- `AttitudeController`
  - consumes angle-to-rate gains and rate limits
- `InputMapper`
  - consumes roll/pitch angle and yaw-rate limits

The starter implementation wires one concrete example into `InputMapper`, which now reads its angle and yaw-rate limits from the registry.

### 5. Persistence

The parameter system should reserve explicit hooks for future persistence:

- `load_from_storage()`
- `save_to_storage()`
- `schema_version()`

In the minimal version the storage hooks remain placeholders, and the schema/version field exists so future stored parameter blobs can be validated against the expected layout.

### 6. Minimal implementation shape

The starter implementation should contain:

- `Parameter`
  - static definition with current value, defaults, and limits
- `ParameterRegistry`
  - singleton-style access to a fixed parameter table
  - direct indexed access by `ParameterId`
- typed accessors
  - `get_float()`, `get_int()`, `get_bool()`
  - `set_float()`, `set_int()`, `set_bool()`
  - range-checked writes
- one real module integration
  - `InputMapper` reading its limits from the registry

This is the minimum practical parameter system before adding communications, persistence backends, or change notification.

## 2.18 Minimal telemetry publishing system

The firmware also needs a low-rate telemetry path so key internal state can be observed during bring-up,
tuning, and debugging without disturbing the fast control loop.

### 1. Goals

The telemetry system should:

- expose internal state for debugging and monitoring
- remain lightweight
- avoid blocking the fast control path

### 2. Data to publish

A minimal telemetry frame should include:

- attitude
  - roll, pitch, yaw
- gyro rates
- rate-loop debug data
  - rate error per axis
  - `P`, `I`, and `D` terms per axis
  - unsaturated and saturated controller outputs
  - saturation flags
- control outputs
  - roll, pitch, yaw, thrust
- motor outputs
- flight state
  - disarmed, armed, failsafe

This is enough to inspect the basic control chain end to end.

### 3. Structure

Use:

- `TelemetryFrame`
  - fixed-size snapshot of the most useful state
- `TelemetryPublisher`
  - small UART-backed publisher

The production-oriented minimum should publish a fixed-size binary frame so runtime cost remains bounded and independent of numeric formatting.
For stream robustness, that payload should be wrapped in a small fixed header such as `[sync][size][crc][payload]`.
The on-wire packet should use a packed layout and explicit little-endian encoding so the frame is stable across toolchains and MCU targets.

### 4. Behavior

The publisher should:

- collect state from estimator, sensing, control, mixer output, and safety supervisor
- run at a lower rate than the control loop
- a practical starter rate is `50 Hz`

This keeps observability decoupled from high-rate control execution.

### 5. Integration

- runs as a scheduled task in a background service class
- uses the HAL `IUartPort` interface
- should attempt one bounded write per telemetry tick

The publisher should perform one fixed-size write with no retries, no heap allocation, and no dynamic string formatting.
The HAL UART backend is expected to map that call to a bounded transmit path such as a small driver FIFO or DMA-backed enqueue.
The telemetry task should never rely on blocking UART drain time; `write()` success only means the packet was accepted by the driver path, while `busy` or `io_error` should be treated as immediate publish failure for that tick.
One practical starter layout is:

- `sync`
  - `0xAA55`
- `size`
  - fixed payload size
- `crc`
  - fixed-cost CRC32 over the payload
- `payload`
  - existing fixed `BinaryTelemetryFrame`

The payload may also include a monotonic sequence counter for bring-up and loss-detection debugging, as long as the total packet size remains fixed.

### 6. Minimal implementation shape

The starter implementation should contain:

- `TelemetryFrame`
  - timestamp plus selected estimator, sensor, control, output, and safety fields
- `TelemetryPublisher`
  - `publish()` method using `IUartPort`
- scheduler integration
  - low-rate task, for example every `20000 us`

This is enough to publish useful state without introducing protocol stacks, queues, or dynamic memory.

## 2.19 Minimal flight logging system

The firmware also needs a small logging path so telemetry data can be retained for offline analysis
without blocking the control or telemetry loops.

### 1. Goals

The logging system should:

- persist useful state for later inspection
- avoid blocking high-rate tasks
- remain deterministic and heap-free

### 2. Data

The simplest starting point is to reuse `TelemetryFrame` directly as the logging record.
That is especially useful once the telemetry snapshot carries rate-loop debug terms, because the same fixed record can support both live tuning and offline PID analysis.

That avoids:

- duplicate data definitions
- extra packing code during the initial bring-up stage

A more compact binary record can be introduced later if storage bandwidth becomes a concern.

### 3. Structure

Use:

- `LogBuffer`
  - fixed-size ring buffer of telemetry frames
- `Logger`
  - accepts pushed frames and flushes them to a storage backend
- `ILogStorage`
  - abstract append interface for future flash or SD implementations

### 4. Behavior

Minimal behavior:

- telemetry task pushes frames into the ring buffer
- a lower-rate background task flushes one buffered frame per tick to storage
- if the buffer fills, drop the oldest frame and retain the newest

Dropping oldest data under pressure is a simple, deterministic overflow policy that avoids blocking.

### 5. Storage

Storage should be abstracted behind:

- `ILogStorage::append()`

The minimal implementation can use a stub storage backend, with real flash or SD support added later.

### 6. Minimal implementation shape

The starter implementation should contain:

- `LogBuffer`
  - static ring buffer with push/pop
- `Logger`
  - `push()` and `flush()`
- `ILogStorage`
  - append-only storage abstraction
- scheduler integration
  - telemetry task pushes frames
  - background flush task drains them gradually

This is the minimum practical logging system before adding file formats, log rotation, or bulk asynchronous storage handling.

## 2.20 Minimal persistent log storage backend

The logger also needs a storage backend that can accept appended records without pulling storage concerns
into the control or telemetry paths.

### 1. Storage options

Two practical embedded options are:

- flash-based append storage
  - good when no removable media is available
  - requires erase-page management and wear considerations
- SD card append storage
  - simpler conceptual model for early bring-up
  - naturally file-oriented and suitable for larger logs

For the minimal skeleton, an SD-style append backend is the simpler choice.

### 2. Interface

The backend should implement `ILogStorage`.

A minimal SD-style backend can wrap a lower-level append device interface:

- `IAppendStorageDevice`
  - `initialize()`
  - `append_bytes()`
- `SdLogStorage`
  - implements `ILogStorage::append()`

This keeps the logger independent from the actual medium.

### 3. Behavior

Required behavior:

- append-only writes
- no storage operations in the control path
- write failures handled safely

Minimal policy:

- telemetry task only pushes frames into the log buffer
- flush task calls `append()` in background context
- if storage write fails, mark the backend unhealthy and stop claiming success

This keeps failure handling bounded and explicit.

### 4. Constraints

The backend should remain:

- heap-free
- bounded in write time
- best-effort power-loss tolerant

A practical first step is to write one fixed-size record per append. If power is lost during a write, offline tools can recover by scanning for valid record headers and lengths.

### 5. Minimal implementation shape

The starter implementation should contain:

- `SdLogStorage`
  - append-only backend implementing `ILogStorage`
- `IAppendStorageDevice`
  - abstract write target for future SD or flash drivers
- fixed binary record
  - includes a magic value, version, payload fields, and record size
- initialization
  - explicit `initialize()` call before logging begins

This is the minimum practical persistent logging backend before adding real SD filesystems, flash page management, or recovery indexing.

## 3. Proposed project layout

This layout avoids mirroring ArduPilot and instead uses product-style domains around a runtime core.

```text
DroneFirmware/
  docs/
    architecture.md
  cmake/
  src/
    app/
      main.cpp
    common/
      TimeTypes.hpp
    runtime/
      Scheduler.hpp
      Scheduler.cpp
    platform/
      Hal.hpp
    sensing/
      Sensor.hpp
    control/
      ControlLoop.hpp
      ControlLoop.cpp
    future/
      Placeholder.md
  tests/
  tools/
  boards/
  integrations/
```

### Layout rationale

- `app`: startup composition and dependency wiring
- `common`: small shared types with low coupling
- `runtime`: deterministic execution engine and lifecycle services
- `platform`: pure abstraction interfaces plus later board adapters
- `sensing`: device-agnostic sensor contracts and aggregation
- `control`: control pipeline surfaces and loop implementations
- `boards`: concrete MCU and carrier-board targets
- `integrations`: adapters for Drone Factory, telemetry stacks, and external tooling
- `future`: reserved area for autonomy and maneuver intelligence experiments kept away from the safety-critical baseline
