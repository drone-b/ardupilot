# ArduPilot Capability Mapping and DroneFirmware Development Roadmap

## Purpose

ArduPilot has a level of technical maturity that is vastly higher than the current `DroneFirmware`
baseline. That is expected.

ArduPilot has:

- more than a decade of iteration
- broad field exposure
- support for many vehicle classes
- support for many boards, sensors, and actuators
- large operational feedback loops from real deployments
- a deep body of tuning, safety, logging, and test practices

`DroneFirmware` should not try to "match ArduPilot quickly".
Instead, it should systematically extract the *capabilities*, *engineering patterns*, and *development
disciplines* that made ArduPilot robust, then convert those into a staged proprietary roadmap for
`DroneFirmware` / `DroneOS`.

This document does exactly that.

## Important Boundary

This roadmap is based on:

- architectural concepts
- capability mapping
- engineering patterns
- system-level lessons

It is **not** a request to copy code, file organization, APIs, or GPL implementation details.

The correct use of ArduPilot as reference is:

- understand what mature autopilot firmware must be able to do
- understand what technical techniques are repeatedly used in that domain
- decide what original `DroneFirmware` implementation should be built next

## 1. Executive Assessment

Today, `DroneFirmware` is strongest in:

- architectural clarity
- explicit module boundaries
- deterministic control-path thinking
- simple, inspectable code
- clean contracts for scheduler, HAL, sensors, control, safety, telemetry, logging, and parameters

Today, ArduPilot is much stronger in:

- estimator maturity
- sensor breadth and robustness
- actuator/output breadth
- fault handling depth
- mode richness
- mission logic
- tuning tooling
- log and telemetry ecosystem
- board support
- validation and autotest coverage
- field-proven behavior under edge cases

That means the right strategy is:

1. keep the `DroneFirmware` architecture clean
2. expand capability depth in a controlled order
3. use ArduPilot maturity as a benchmark, not as a template

## 2. Capability Map

This section maps major technical domains.

### 2.1 Runtime and Scheduling

#### ArduPilot maturity

ArduPilot demonstrates mature handling of:

- fixed-rate main-loop execution
- budgeted scheduled tasks
- runtime load monitoring
- task timing instrumentation
- long-term operational stability under mixed workloads
- board-specific timing adaptation

#### Techniques worth learning from

- release-based scheduling
- strict separation between interrupt signaling and scheduled work
- performance monitoring of loop timing
- graceful degradation under CPU pressure
- high-rate control isolation from low-rate services

#### DroneFirmware current state

Already implemented:

- cooperative fixed-capacity scheduler
- priority-class execution
- budget and overrun instrumentation
- skipping of stale periodic releases
- slack-based denial of lower-priority work
- explicit pipeline ordering

#### Gap

Still missing:

- long-term load statistics
- watchdog integration tied to scheduler health
- fault escalation based on repeated overruns
- per-task runtime export through telemetry/logging
- board-specific performance validation

#### Priority

High.
The runtime model is already good enough to be the platform foundation, so it should now be hardened
through validation and observability rather than redesigned.

### 2.2 HAL and Board Support

#### ArduPilot maturity

ArduPilot has deep experience in:

- many MCU targets and boards
- UART/SPI/I2C/CAN integration
- DMA-backed drivers
- RC input/output integration
- storage integration
- clock/watchdog/reset behavior
- board-definition systems

#### Techniques worth learning from

- capability-based HAL contracts
- board-specific adaptation below common interfaces
- DMA-first thinking for high-rate paths
- explicit bus ownership and buffering semantics
- declarative board configuration

#### DroneFirmware current state

Already implemented:

- clean HAL interfaces
- explicit non-blocking UART contract
- SPI/I2C/UART/PWM abstractions
- interrupt-router abstraction

#### Gap

Still missing:

- real STM32 implementation layer
- board definition scheme
- DMA-backed transfer paths
- robust RX/TX buffering
- CAN/UAVCAN/DroneCAN strategy
- RC input path
- real persistent storage backend

#### Priority

Very high.
Without a real HAL/board layer, the rest of the system cannot become operational.

### 2.3 Sensor Stack

#### ArduPilot maturity

ArduPilot is significantly ahead in:

- support for many IMUs and external sensors
- redundancy handling
- health checks
- calibration flows
- consistency checks across sensors
- vibration handling
- high-rate buffered acquisition

#### Techniques worth learning from

- sensor-manager ownership of device fleet
- explicit health and validity state
- timing-aware sampling
- latest-sample plus buffered-history patterns where needed
- calibration as a first-class subsystem

#### DroneFirmware current state

Already implemented:

- `SensorManager`
- timestamped IMU sample contract
- latest-stable sample publication
- sample validity and freshness checks

#### Gap

Still missing:

- accelerometer/gyro calibration
- magnetometer support
- barometer support
- GNSS support
- redundant sensor arbitration
- vibration metrics
- sensor-specific fault taxonomy

#### Priority

Very high.
The next operational leap depends on broadening the sensor layer beyond the single IMU path.

### 2.4 Estimation

#### ArduPilot maturity

This is one of ArduPilot's biggest advantages.
It includes:

- AHRS layers
- multiple EKF variants
- source switching
- bias estimation
- robust handling of degraded sensing
- support for GPS, baro, mag, airspeed, range, visual odometry, and more

#### Techniques worth learning from

- layered estimator architecture
- fusion-source abstraction
- estimator validity and lane switching
- health-aware output contracts
- estimator logging and debug observability

#### DroneFirmware current state

Already implemented:

- minimal complementary-filter attitude estimator
- timestamp-based `dt`
- estimator validity bit

#### Gap

Still missing:

- yaw observability beyond gyro integration
- velocity and position estimation
- bias estimation
- multi-sensor fusion
- degraded-mode estimation
- estimator reset/realignment policy

#### Priority

Very high.
Estimator maturity is one of the largest gaps between a clean prototype and a robust autopilot.

### 2.5 Control System

#### ArduPilot maturity

ArduPilot has mature:

- inner/outer loop control structures
- vehicle-specific attitude control variants
- rate shaping
- tuning support
- mode-specific control policies
- actuator-aware limits and nonlinear compensation

#### Techniques worth learning from

- cascaded loops
- command shaping
- axis-specific control policy
- saturation-aware control
- logging of internal controller states

#### DroneFirmware current state

Already implemented:

- inner rate loop
- outer attitude loop
- anti-windup
- derivative-on-measurement
- pilot input mapping
- flight-mode branching for `STABILIZE` and `ACRO`
- controller debug observability

#### Gap

Still missing:

- heading hold
- altitude/vertical control
- position and velocity control
- richer command shaping
- gain scheduling
- vehicle-specific controller variants
- autotuning workflows

#### Priority

High.
The control architecture is already well chosen; the next step is to expand it in depth, not replace it.

### 2.6 Actuator Allocation and Output

#### ArduPilot maturity

ArduPilot is much more advanced in:

- mixer breadth
- motor geometry support
- thrust linearization
- spool logic
- lost-motor handling
- multiple actuator classes
- helicopter and hybrid actuation systems

#### Techniques worth learning from

- geometry-driven allocation
- saturation handling that preserves authority
- output protocol abstraction
- safe disarm behavior
- output-test and diagnostic tooling

#### DroneFirmware current state

Already implemented:

- simple quad-X mixer
- proportional desaturation
- PWM output path with gating

#### Gap

Still missing:

- multiple frame geometries
- servo-based vehicles
- DShot backend behavior
- motor protocol configuration
- thrust linearization
- actuator diagnostics
- lost-actuator strategies

#### Priority

Medium-high for multirotor productization.
Very high if the roadmap really includes ground and marine platforms from the start.

### 2.7 Safety, Arming, and Failsafe

#### ArduPilot maturity

ArduPilot is particularly strong in:

- layered arming checks
- many failsafe sources
- mode-specific responses
- user-visible fault reporting
- long-tail handling of operational edge cases

#### Techniques worth learning from

- explicit safety state machine
- latched fault handling
- disarm authority above normal control
- health-based gating
- event logging for safety transitions

#### DroneFirmware current state

Already implemented:

- `DISARMED`, `ARMED`, `FAILSAFE`
- explicit block reasons
- latched failsafe
- arm/disarm through unified pilot command
- debounced transition into failsafe

#### Gap

Still missing:

- battery failsafe
- link-loss failsafe
- estimator-specific policy escalation
- geofence/fault coupling
- event logging of safety transitions
- pre-arm reporting with actionable diagnostics

#### Priority

Very high.
This is a product-critical area and one of the easiest places to gain real robustness quickly.

### 2.8 Telemetry and Communication

#### ArduPilot maturity

ArduPilot is much stronger in:

- MAVLink ecosystem integration
- ground-station interoperability
- parameter transport
- mission transport
- stream-rate management
- protocol routing and link management

#### Techniques worth learning from

- explicit transport budgets
- payload-size awareness
- bounded serial behavior
- protocol/transport separation
- strong operator-facing observability

#### DroneFirmware current state

Already implemented:

- fixed-size binary telemetry frame
- framed transport with sync/size/CRC
- packed little-endian wire encoding
- bounded UART contract

#### Gap

Still missing:

- external protocol definition
- parameter exchange protocol
- mission protocol
- configuration tooling integration
- link management and stream control
- companion-computer control path

#### Priority

High.
DroneFirmware will need this early because the product vision explicitly depends on external platform integration.

### 2.9 Logging and Diagnostics

#### ArduPilot maturity

ArduPilot is extremely strong in:

- binary logs
- event logging
- subsystem-specific log messages
- post-flight analysis support
- tuning workflows built around logs

#### Techniques worth learning from

- structured binary records
- stable log schemas
- event/error separation
- controller and estimator introspection fields
- bounded write paths

#### DroneFirmware current state

Already implemented:

- fixed-size ring-buffer logger
- append-only persistent backend abstraction
- telemetry-frame reuse for logging
- controller internals exposed for tuning/debug

#### Gap

Still missing:

- explicit event log messages
- schema/version strategy for logs
- log extraction tooling
- parser/visualization tooling
- retention/rotation strategy

#### Priority

High.
Logging is essential for tuning, validation, and field support.

### 2.10 Parameters and Configuration

#### ArduPilot maturity

ArduPilot is much stronger in:

- extensive parameter metadata
- persistence
- visibility rules
- compatibility over time
- integration with GCS and tooling

#### Techniques worth learning from

- schema-aware persistence
- strong metadata around limits/defaults
- grouping and namespacing
- stable compatibility rules

#### DroneFirmware current state

Already implemented:

- direct indexed registry
- type-safe access
- min/max/default metadata
- schema version placeholder

#### Gap

Still missing:

- units and display metadata
- persistence implementation
- change notification
- parameter groups/profiles
- remote tuning workflow

#### Priority

Medium-high.
This becomes critical as soon as tuning moves beyond source-code edits.

### 2.11 Mission, Guidance, and Autonomy

#### ArduPilot maturity

ArduPilot is vastly ahead in:

- waypoint missions
- guidance logic
- avoidance behaviors
- return modes
- terrain-aware navigation
- many vehicle-specific mission behaviors

#### Techniques worth learning from

- layered separation between mission, guidance, and stabilization
- explicit mode-dependent mission behavior
- bounded autonomous task scheduling
- fault-aware mission interruption and resume behavior

#### DroneFirmware current state

Already implemented:

- basic mode selection
- manual pilot-command path
- minimal cascaded control stack

#### Gap

Still missing almost everything in this domain:

- mission engine
- navigation targets
- path-following
- terrain-aware planning
- recovery behaviors
- autonomy-task execution model

#### Priority

Medium in the near term.
Very high in the strategic roadmap.

### 2.12 Vehicle Breadth

#### ArduPilot maturity

ArduPilot supports:

- multirotors
- helicopters
- fixed-wing
- quadplanes
- rovers
- subs
- blimps
- tracker/peripheral firmware

#### DroneFirmware current state

Current implementation strongly resembles:

- multirotor first

#### Gap

Still missing:

- abstraction boundaries for non-multirotor actuation
- guidance and control variants for ground/marine
- vehicle-profile system

#### Priority

Strategic.
Do not generalize too early, but keep the architecture ready.

### 2.13 Tooling, Validation, and Test Discipline

#### ArduPilot maturity

This is one of the most important gaps.
ArduPilot has:

- SITL
- autotest infrastructure
- board builds
- parameterized test coverage
- field log workflows

#### Techniques worth learning from

- simulation-backed regression
- repeatable flight scenarios
- configuration-based test suites
- logs as test artifacts
- vehicle-specific validation matrices

#### DroneFirmware current state

Already implemented:

- architecture and compile-level validation
- deterministic design choices that support future testability

#### Gap

Still missing:

- simulator integration
- module unit tests
- hardware-in-loop strategy
- replay/log analysis tools
- regression suites

#### Priority

Extremely high.
Without this, maturity will remain slow and risky regardless of architecture quality.

## 3. Development Techniques Worth Adopting as First-Class Principles

These are not "features". They are engineering habits that should become part of `DroneFirmware`.

### 3.1 Timing First

- every critical task has a budget
- every critical data path is timestamped
- no blocking operations in the fast path
- lower-priority work must be droppable

### 3.2 Health First

- every major subsystem publishes validity/health
- control never assumes data is valid
- safety policy sits above nominal control

### 3.3 Logging First

- every important internal state should be observable
- controller tuning should rely on logs, not guesswork
- safety transitions should be logged

### 3.4 Configuration Safety

- all tunable behavior goes through validated parameters
- ranges/defaults/schema are part of the system contract

### 3.5 Vehicle Separation

- generic infrastructure stays separate from vehicle-specific behavior
- control allocation is not the same thing as vehicle guidance

### 3.6 Testability as Architecture

- simulation should be treated as a product capability
- replay/log-driven validation should be designed in early

## 4. Recommended DroneFirmware Roadmap

## Phase 0: Current Baseline

Already present:

- HAL contracts
- scheduler
- sensor manager
- minimal estimator
- cascaded rate/attitude control
- mixer
- safety supervisor
- parameters
- telemetry
- logging

This is the correct architectural seed.

## Phase 1: Product-Grade Core Stabilization

Build next:

- real STM32 HAL implementation
- real UART/SPI/I2C drivers with DMA-aware behavior
- IMU calibration
- barometer and magnetometer support
- battery monitoring
- safety event logging
- scheduler stats in telemetry/logs

Reason:
This phase turns the current skeleton into a usable stabilization platform.

## Phase 2: Estimation and Vehicle Robustness

Build next:

- better yaw estimation
- velocity/altitude estimation
- estimator resets and fault handling
- sensor redundancy concepts
- richer arming checks
- RC input subsystem

Reason:
This phase closes one of the biggest gaps versus mature autopilot behavior.

## Phase 3: Configuration, Tuning, and Diagnostics

Build next:

- persistent parameters
- parameter units/metadata
- parameter change notifications
- richer logging schema
- offline log parser/analysis tools
- tuning workflow support

Reason:
This phase makes field tuning operational instead of code-centric.

## Phase 4: External Platform Integration

Build next:

- command/telemetry protocol strategy
- Drone Factory integration layer
- remote configuration channel
- mission upload/download model
- secure command authority model

Reason:
This is where `DroneOS` becomes part of a larger product ecosystem.

## Phase 5: Guidance and Mission System

Build next:

- waypoint mission engine
- position/velocity control
- return behaviors
- geofence integration
- terrain-aware navigation

Reason:
This phase moves from "stable vehicle" to "operational autonomous platform".

## Phase 6: Validation and Scale

Build next:

- SITL or equivalent simulator path
- replay infrastructure
- autotest matrix
- board qualification matrix
- long-duration stress tests

Reason:
This is what converts implementation into maturity.

## Phase 7: Multi-Vehicle and Advanced Intelligence

Build later:

- vehicle profiles for air/ground/marine
- task models for heavier autonomy
- companion-compute interfaces
- AI-assisted maneuver modules

Reason:
This should come after the core stack is already trustworthy.

## 5. What DroneFirmware Should Not Do

To stay disciplined, `DroneFirmware` should avoid:

- copying ArduPilot implementation structures directly
- trying to support all vehicle types too early
- adding protocol complexity before the HAL and estimator are strong
- adding autonomy before logging, parameters, and validation are mature
- expanding features faster than testability

## 6. Recommended Next Concrete Deliverables

The most rational next deliverables are:

1. real STM32 HAL skeleton and one board port
2. battery/power monitoring contract
3. magnetometer and barometer integration
4. scheduler and safety stats in telemetry/logging
5. persistent parameter storage
6. simulator/test harness foundation

## 7. Final View

ArduPilot should be treated as:

- evidence of what mature autopilot firmware must eventually support
- evidence of what engineering disciplines matter most
- a benchmark for capability depth

`DroneFirmware` should be treated as:

- a clean, modern, proprietary architecture baseline
- a long-horizon product platform
- a system that should grow by disciplined capability layering

The correct roadmap is not:
"rebuild ArduPilot quickly".

The correct roadmap is:
"systematically build the minimum set of mature capabilities, in the right order, with a cleaner architecture and strong observability from day one."
