# DroneOS Master Development Plan

Navigation: [Docs Index](README.md) | [Architecture](architecture.md) | [Technical Concepts](technical-concepts.md) | [Glossary](glossary.md)

Status: Draft v1  
Scope: Development plan derived from ArduPilot capability mapping, rewritten as an original DroneOS roadmap

---

## 1. Purpose

DroneOS should grow from a clean proprietary architecture into a production-grade flight-control platform.
ArduPilot is useful as a maturity benchmark because it demonstrates which capabilities matter after many
years of real-world use across many vehicles, boards, sensors, and operating conditions.

This plan converts that maturity mapping into execution milestones for DroneOS.

This document is not a code-copying plan.
It is a capability and engineering-discipline plan.

---

## 2. Planning Principles

1. Keep the architecture clean and original.
2. Prioritize flight safety and deterministic execution before feature breadth.
3. Build observability before advanced autonomy.
4. Treat parameters, logs, and tests as product features, not support utilities.
5. Add vehicle types only after the core runtime, HAL, sensing, estimation, control, and safety contracts are stable.
6. Use ArduPilot as a benchmark for maturity, not as an implementation template.

---

## 3. Milestone Overview

| Milestone | Theme | Priority | Main Outcome |
|---|---|---:|---|
| M0 | Architecture Baseline | Complete / ongoing | Maintain clean modular foundation |
| M1 | Deterministic Runtime Core | Critical | Scheduler, timing, and observability are trustworthy |
| M2 | STM32 HAL and Board Bring-up | Critical | Firmware runs on real flight-controller hardware |
| M3 | Sensor Stack Hardening | Critical | IMU, barometer, magnetometer, GNSS, and health contracts |
| M4 | Estimation Upgrade | Critical | Robust attitude, yaw, altitude, and velocity foundation |
| M5 | Safety and Arming Hardening | Critical | Production-oriented fault gating and event visibility |
| M6 | Control and Actuation Productization | High | Tunable, logged, actuator-aware stabilization |
| M7 | Parameters, Logging, and Tuning | High | Real field-tuning workflow |
| M8 | External Platform Integration | High | Drone Factory / companion / command links |
| M9 | Mission and Guidance Layer | Medium-high | Autonomous waypoint and recovery behavior |
| M10 | Simulation and Validation | Critical | SITL-style repeatable regression testing |
| M11 | Multi-Vehicle Expansion | Strategic | Ground and marine support without core rewrite |
| M12 | Autonomous Maneuver Intelligence | Strategic | Higher-level intelligent behavior built on stable core |

---

## 4. Milestone Details

## M0: Architecture Baseline

### Goal

Preserve the current modular foundation while continuing implementation.

### Current State

Already present:

- HAL interfaces
- cooperative fixed-capacity scheduler
- sensor manager
- minimal IMU contract
- complementary attitude estimator
- cascaded attitude/rate control
- quad-X mixer
- safety supervisor
- parameter registry
- framed binary telemetry
- fixed-size logging path
- ArduPilot glossary and capability-mapping documentation

### Dependencies

None.

### Risk

Low.
Main risk is architectural drift as more features are added.

### Acceptance Criteria

- Core module boundaries remain clear.
- No fast-path dynamic allocation is introduced.
- No ArduPilot code or GPL structure is copied.
- Documentation stays aligned with implemented contracts.

---

## M1: Deterministic Runtime Core

### Goal

Make runtime behavior measurable, bounded, and enforceable enough for flight-control use.

### Work Items

- Expose scheduler timing stats through telemetry/logging. (Implemented in the fixed binary frame and binary log record.)
- Add scheduler health summary.
- Add repeated-overrun escalation hooks.
- Add watchdog policy tied to scheduler health.
- Add runtime mode concept if needed:
  - normal
  - degraded
  - emergency

### Dependencies

- Existing scheduler instrumentation
- Existing telemetry/logging path
- Safety supervisor block-reason system

### Risk

Medium.
Incorrect degradation policy can mask real timing bugs or accidentally starve useful diagnostics.

### Acceptance Criteria

- Each task exposes:
  - last execution time
  - maximum execution time
  - overrun count
  - skipped release count
  - slack denial count
- Background work cannot delay the control path.
- Scheduler health can be logged for offline analysis.
- Repeated runtime violations can be surfaced to safety or diagnostics without changing control math.

---

## M2: STM32 HAL and Board Bring-up

### Goal

Run the firmware on a real STM32-class flight controller through an original HAL implementation.

### Work Items

- Create `platform/stm32/` implementation layer.
- Define a minimal board configuration format.
- Implement monotonic microsecond timer.
- Implement watchdog and reset control.
- Implement SPI with bounded transfer semantics.
- Implement I2C with timeout and bus recovery policy.
- Implement UART with non-blocking FIFO or DMA enqueue semantics.
- Implement PWM output bank.
- Implement sensor interrupt routing.
- Add one reference board port.

### Dependencies

- HAL interfaces
- scheduler runtime
- sensor manager
- safety output gating

### Risk

High.
Hardware timing, DMA, interrupt priorities, and peripheral conflicts can destabilize the system.

### Acceptance Criteria

- The firmware boots on one STM32 target.
- The monotonic clock is stable and shared between ISR and task contexts.
- SPI IMU read path is bounded.
- UART telemetry write is enqueue-only and never blocks on full physical transmission.
- PWM outputs can be configured, written, and disarmed safely.
- Watchdog is serviced only through a deliberate runtime path.

---

## M3: Sensor Stack Hardening

### Goal

Move from a single minimal IMU path to a health-aware sensor subsystem.

### Work Items

- Add IMU calibration records:
  - gyro bias
  - accelerometer bias
  - accelerometer scale
- Add barometer abstraction and sample contract.
- Add magnetometer abstraction and sample contract.
- Add GNSS abstraction and sample contract.
- Add sensor health state per device.
- Add sensor fault reasons.
- Add sample-rate monitoring.
- Add sensor timing metrics to logs.

### Dependencies

- STM32 HAL
- timestamp contract
- parameter system
- logging system

### Risk

High.
Bad calibration or inconsistent timing can silently degrade control and estimation.

### Acceptance Criteria

- IMU samples include calibrated values and health status.
- Sensor invalidity blocks control or arming through safety policy.
- Barometer and magnetometer can be initialized and monitored independently.
- GNSS has a typed sample contract but does not need full navigation fusion yet.
- Sensor timing data can be logged.

---

## M4: Estimation Upgrade

### Goal

Build a more robust state-estimation foundation without jumping immediately to a full EKF.

### Work Items

- Replace Euler attitude state with quaternion attitude state if not already done.
- Add gyro bias estimation or correction path.
- Add yaw correction using magnetometer or external heading source.
- Add altitude estimate from barometer.
- Add vertical velocity estimate.
- Add estimator reset policy.
- Add estimator debug telemetry/log fields.
- Define future EKF/ESKF interface boundary.

### Dependencies

- IMU calibration
- barometer support
- magnetometer support
- sensor health contracts
- logging/telemetry observability

### Risk

High.
Estimator instability can directly affect control stability and arming decisions.

### Acceptance Criteria

- Attitude estimate is valid, timestamped, and logged.
- Yaw drift is bounded when a valid heading source exists.
- Altitude estimate is available with validity state.
- Estimator invalidity is propagated into safety gating.
- The estimator interface can later support EKF/ESKF without rewriting control modules.

---

## M5: Safety and Arming Hardening

### Goal

Make safety behavior production-oriented and diagnosable.

### Work Items

- Add battery/power validity inputs.
- Add command-link timeout policy.
- Add RC-link timeout policy when RC input exists.
- Add estimator-specific safety reasons.
- Add sensor-specific safety reasons.
- Add pre-arm diagnostic reporting.
- Add safety transition event logging.
- Add failsafe action matrix.

### Dependencies

- command validity
- battery/power monitor
- sensor health
- estimator health
- event logging

### Risk

High.
Overly permissive safety policy is dangerous; overly strict policy prevents operation.

### Acceptance Criteria

- Safety supervisor reports clear block reasons.
- Failsafe remains latched until explicit safe recovery.
- Disarm always overrides normal control.
- Pre-arm failures are visible in telemetry/logs.
- Each safety transition creates an event record.

---

## M6: Control and Actuation Productization

### Goal

Make stabilization tunable, diagnosable, and actuator-aware.

### Work Items

- Move all rate and attitude gains into parameters.
- Add yaw hold or heading-hold behavior.
- Add vertical/throttle control path.
- Add actuator saturation feedback into anti-windup.
- Expand mixer into a generic control allocator.
- Add frame geometry definitions.
- Add DShot or digital ESC protocol path.
- Add motor test mode with safety gating.

### Dependencies

- parameter system
- logging/telemetry
- HAL output backend
- safety supervisor

### Risk

Medium-high.
Control changes can destabilize the vehicle quickly if not paired with logging and tests.

### Acceptance Criteria

- Controller logs contain error, P/I/D, saturated and unsaturated outputs.
- Gains can be tuned through validated parameters.
- Motor output saturation is observable.
- Quad-X behavior remains stable after allocator changes.
- Control behavior remains deterministic and heap-free.

---

## M7: Parameters, Logging, and Tuning

### Goal

Create a real tuning workflow instead of requiring source-code edits.

### Work Items

- Add persistent parameter storage.
- Add units and display metadata.
- Add parameter groups.
- Add change notification hooks.
- Add log schema versioning.
- Add offline log parser.
- Add tuning plot scripts.
- Add safety/event log stream.

### Dependencies

- parameter registry
- persistent storage backend
- binary log format
- telemetry protocol

### Risk

Medium.
Bad persistence or schema migration can corrupt configuration.

### Acceptance Criteria

- Parameters persist across reboot.
- Parameter writes are range-checked and type-checked.
- Log records have version/schema information.
- Offline tooling can decode controller and scheduler data.
- Tuning can be performed from logs without recompilation.

---

## M8: External Platform Integration

### Goal

Integrate DroneOS with Drone Factory and future companion/control platforms through bounded, explicit protocols.

### Work Items

- Define command protocol.
- Define telemetry protocol versioning.
- Define parameter transport.
- Define command authority and timeout rules.
- Add companion link abstraction.
- Add Drone Factory adapter layer.
- Add security and identity plan.

### Dependencies

- telemetry framing
- parameter registry
- safety command gating
- command freshness logic

### Risk

High.
External control paths must not bypass safety or destabilize timing.

### Acceptance Criteria

- External commands enter through one command path.
- Command validity includes freshness and authority.
- Link failure leads to deterministic safety behavior.
- Protocol handling does not block fast control tasks.
- Drone Factory integration remains outside low-level control modules.

---

## M9: Mission and Guidance Layer

### Goal

Add autonomous navigation behavior above the stabilization layer.

### Work Items

- Add mission item model.
- Add waypoint target representation.
- Add guidance target generator.
- Add return-home behavior.
- Add geofence model.
- Add terrain-awareness placeholder.
- Add mission state logging.

### Dependencies

- position/velocity/altitude estimation
- safety supervisor
- parameter persistence
- external command protocol

### Risk

High.
Navigation errors can produce unsafe high-level commands even when low-level stabilization is correct.

### Acceptance Criteria

- Guidance produces bounded motion targets.
- Mission logic cannot bypass safety gating.
- Mission state is logged.
- Return-home behavior has clear preconditions and failure modes.

---

## M10: Simulation and Validation

### Goal

Create repeatable regression testing before broad flight testing.

### Work Items

- Add minimal SITL-style simulation harness.
- Add deterministic sensor replay path.
- Add unit tests for scheduler, parameters, safety, mixer, and control.
- Add log replay tooling.
- Add CI build for host simulation.
- Add hardware smoke-test plan.

### Dependencies

- stable module interfaces
- logging
- scheduler stats
- parameter schema

### Risk

Medium.
Weak simulation can create false confidence; validation must be tied to real logs and hardware tests.

### Acceptance Criteria

- Core modules have unit tests.
- Host simulation can run the scheduler and control loop.
- Logs can be replayed or decoded.
- CI catches regressions in safety and timing logic.
- Manual bench-test checklist exists for first hardware tests.

---

## M11: Multi-Vehicle Expansion

### Goal

Extend DroneOS beyond multirotors without contaminating the core architecture.

### Work Items

- Add vehicle profile model.
- Add ground vehicle actuation contract.
- Add marine vehicle actuation contract.
- Add fixed-wing/hybrid feasibility study.
- Add profile-specific safety constraints.
- Add mode availability matrix per vehicle type.

### Dependencies

- mature core HAL
- stable safety supervisor
- generalized actuator allocation
- mission/guidance abstraction

### Risk

Medium-high.
Premature generalization can make the core system complex before the multirotor path is mature.

### Acceptance Criteria

- Vehicle-specific code does not enter generic runtime/HAL modules.
- Each vehicle profile declares its supported modes and actuator model.
- Non-multirotor support can be compiled out or isolated.

---

## M12: Autonomous Maneuver Intelligence

### Goal

Add higher-level maneuver intelligence only after the core safety and control stack is reliable.

### Work Items

- Define maneuver command interface.
- Define autonomy task budget.
- Add autonomy sandboxing rules.
- Add AI-assisted command validity constraints.
- Add higher-level decision logging.
- Add operator override model.

### Dependencies

- stable control stack
- mission/guidance layer
- external platform integration
- strong logging and replay
- safety authority model

### Risk

Very high.
Autonomy can generate unsafe goals if safety, validation, and command authority are not mature first.

### Acceptance Criteria

- Autonomous maneuvers cannot bypass safety supervisor.
- AI or companion outputs are time-bounded and authority-bounded.
- Every autonomy decision is observable in logs.
- Manual/operator override remains deterministic.

---

## 5. Cross-Cutting Acceptance Criteria

These criteria apply to every milestone:

- No fast-path heap allocation.
- No unbounded blocking I/O in the control path.
- No safety bypasses from external command sources.
- Every new subsystem exposes health/validity state.
- Every tunable behavior uses validated parameters.
- Every safety-relevant behavior is logged.
- Every new control behavior has telemetry or log observability.
- Every new feature has at least one test strategy:
  - unit test
  - simulation test
  - bench test
  - hardware-in-loop test
  - field-test checklist

---

## 6. Recommended Immediate Execution Order

The next practical development sequence is:

1. Add scheduler stats to telemetry/logging.
2. Add battery/power monitoring contract. (Initial HAL ADC + PowerMonitor task added; telemetry/logging only, not yet wired into safety.)
3. Add persistent parameter storage. (Initial schema-checked binary storage added for host/simulation; future STM32 backend still needed.)
4. Add STM32 HAL skeleton and one reference board port. (Initial STM32H743VIT reference profile, injectable STM32 timebase contract, separated STM32 simulation backend, timer timebase placeholder, watchdog/reset observability, non-blocking UART FIFO with DMA-burst state/metrics and bounded aggregate service hook, telemetry publish backpressure counters, safe PWM bank, bounded SPI/I2C transfer, fixed-capacity STM32 bus factory validation, sensor IRQ simulation, and separated fake STM32 IMU/barometer/magnetometer paths added; real drivers pending.)
5. Add IMU calibration and sensor-health state. (Initial IMU bias/scale application and health counters added; calibration procedure still pending.)
6. Add barometer and magnetometer interfaces. (Initial device/sample contracts added; SensorManager optional acquisition, buffering, sequencing, and health integration added; real drivers pending.)
7. Add safety event logging. (Initial safety block reason and transition counters added to telemetry/logging.)
8. Add host simulation harness. (Initial host regression binary covers AI link, scheduler, parameters, power monitor, safety, estimator, allocator, and hardware graph.)

This order is intentionally conservative.
It improves product readiness before expanding autonomy or vehicle breadth.

---

## 7. Decision Gates

## Gate A: Before Real Motor Spin

Required:

- real HAL clock validated
- PWM output disarm behavior validated
- safety supervisor blocks outputs when invalid
- scheduler timing counters visible
- IMU sample validity path tested

## Gate B: Before First Tethered Hover

Required:

- IMU calibration
- attitude estimator validity path
- rate-loop debug logging
- arming checks
- battery monitoring
- emergency disarm path

## Gate C: Before Untethered Flight

Required:

- persistent parameters
- safety event logs
- scheduler overload behavior validated
- sensor failure behavior validated
- basic field-test checklist

## Gate D: Before Autonomous Flight

Required:

- position/altitude estimation
- mission/guidance safety gating
- command-link timeout behavior
- return/failsafe behavior
- replay or simulation test coverage

---

## 8. Summary

The strategic lesson from ArduPilot is not that DroneOS should become a clone.

The lesson is that mature autopilot firmware needs:

- deep sensing and estimation
- explicit safety gates
- tunable control
- rich logging
- deterministic runtime behavior
- validated hardware abstraction
- repeatable testing
- field-oriented diagnostics

DroneOS should build those capabilities in a deliberate order while preserving its own clean, proprietary architecture.
