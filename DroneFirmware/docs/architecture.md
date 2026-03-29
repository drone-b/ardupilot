# DroneOS — Architecture Overview

Navigation: [Docs Index](README.md) | [Architecture](architecture.md) | [Technical Concepts](technical-concepts.md) | [Glossary](glossary.md) | [Specification Index](../../../DroneFirmware/specification/README.md) | [ADR Index](../../../DroneFirmware/specification/adr/README.md)

Status: Developer guide  
Updated: 2026-03-29  
Formal spec: [`specification/architecture-v1.md`](../../../DroneFirmware/specification/architecture-v1.md)

---

## 1. What DroneOS Is

DroneOS is a **real-time flight controller firmware** written in C++17 for STM32-class
microcontrollers. It is not an ArduPilot fork. The design starts from a clean slate with
three principles:

1. **Determinism first** — all flight-critical work runs in a fixed-period cooperative executive
   with declared WCET budgets per task. No RTOS. No heap allocation in the fast loop.
2. **AI-native interface** — the external control contract is wrench (body torques + thrust), not
   attitude angles. AI agents command the vehicle in torque space at the same rate the control loop
   runs.
3. **Hardware-graph awareness** — components are discovered, modeled, and fingerprinted at runtime.
   Control geometry is generated from the hardware graph, not hardcoded.

For the full formal description see [`specification/strategy-v1.md`](../../../DroneFirmware/specification/strategy-v1.md).

---

## 2. Execution Model

The deterministic core runs at **500 Hz (2.0 ms major cycle)**. Every cycle, tasks execute in
strict priority order:

```
┌─────────────────────────────────────────────── 2.0 ms cycle ──┐
│  estimation_fast  │  critical_fast  │  mission_normal  │ svc  │
└────────────────────────────────────────────────────────────────┘
```

| Priority class       | Contents                                                 | Budget    |
|----------------------|----------------------------------------------------------|-----------|
| `estimation_fast`    | Sensor latch, AttitudeEstimator                          | ~ 350 µs  |
| `critical_fast`      | ControlLoop, ControlAllocator, SafetyCheck, OutputCommit | ~ 400 µs  |
| `mission_normal`     | FlightMode, parameter reads                              | ~ 500 µs  |
| `service_background` | Telemetry, SD log, ParameterService                      | slack only |

**SchedulerMode** degrades automatically under sustained budget violations:
- `normal` → `degraded` (≥ 3 consecutive violations): drops `service_background`
- `degraded` → `emergency` (≥ 10 consecutive violations): drops `mission_normal` as well

Recovery: 200 consecutive clean cycles return to the previous mode. The Scheduler lives in
`src/runtime/`. The main loop in `src/app/main.cpp` enforces the 2 ms cycle boundary with a
residual sleep.

---

## 3. Subsystem Map

```
  Pilot (RC)  ──────►  FlightMode
  AI agent    ──────►  AiControlLink ──► wrench command
                              │
                        ControlLoop (PID rate loop)
                              │  ControlDemand [τx,τy,τz, Fz]
                        ControlAllocator
                              │  MotorOutputs [0..1 per motor]
                        HAL OutputCommit

  IMU (DMA) ──► AttitudeEstimator ──► AttitudeState
                                           │
                         ControlLoop + AiStateSnapshot (egress)
```

---

## 4. Source Directory Reference

| Directory | Responsibility |
|-----------|---------------|
| `src/app/` | Composition root — wires all subsystems, defines tasks and app context |
| `src/runtime/` | `Scheduler` — fixed-priority cooperative executive, `SchedulerMode` |
| `src/estimation/` | `AttitudeEstimator` — quaternion propagation, gyro bias, innovation gate |
| `src/control/` | `ControlLoop` (PID rate loop), `ControlAllocator` (wrench → motors) |
| `src/comms/` | `AiControlLink` (AI command ingress), `TelemetryPublisher` |
| `src/safety/` | `SafetyManager` — arming preconditions, failsafe state machine |
| `src/sensing/` | `SensorManager`, `ImuSample`, sensor abstractions |
| `src/config/` | `Parameters` — typed parameter registry with validated bounds |
| `src/logging/` | `SdLogStorage` — binary log records, enqueue-or-discard backpressure |
| `src/platform/` | `IHal`, `IClock`, `IAppendStorageDevice`, `SimHal` (simulation) |
| `src/common/` | `TimestampUs`, `DurationUs`, shared primitive types |

---

## 5. Key Data Types on the Critical Path

| Type | Header | Description |
|------|--------|-------------|
| `AttitudeState` | `estimation/AttitudeEstimator.hpp` | roll/pitch/yaw, quaternion, gyro_bias, innovation_norm |
| `ControlDemand` | `control/ControlLoop.hpp` | roll/pitch/yaw rate demands + normalized thrust |
| `MotorOutputs` | `control/ControlAllocator.hpp` | per-motor normalized commands [0..1] |
| `AllocatorStatus` | `control/ControlAllocator.hpp` | unallocated wrench residual, saturation mask |
| `AiWrenchCommandV1` | `comms/AiControlLink.hpp` | tau[3] (Nm), thrust (N), seq, timestamp |
| `TelemetryFrame` | `comms/TelemetryPublisher.hpp` | binary telemetry frame (size static_assert guarded) |
| `ImuSample` | `sensing/Sensor.hpp` | gyro_rad_s[3], accel_m_s2[3], timestamp_us |

For full binary schemas (ContractHeaderV1, CRC, schema versioning) see
[`specification/architecture-contracts-v1.md`](../../../DroneFirmware/specification/architecture-contracts-v1.md).

---

## 6. AI Control Interface

DroneOS exposes a **wrench-space command interface** to AI agents on a companion computer.

**Ingress** (AI → FCU): `AiWrenchCommandV1` — body torques `tau[3]` in Nm + collective
thrust in N, validated in `AiControlLink::ingest_wrench()`.

**Egress** (FCU → AI): `AiStateSnapshotV1` — full attitude, rates, bias, innovation status,
allocator saturation, scheduler mode — emitted every cycle (implementation planned).

**Validation on ingress:**
1. `version == 1`
2. `t_cmd_us <= now_fcu_us` (no future-dated commands)
3. `age <= 25 ms` (freshness window)
4. `seq > last_seq_` (monotonic, no replays)

When no fresh AI command is available the system silently falls back to pilot mode.

See `src/comms/AiControlLink.hpp` and
[`specification/architecture-contracts-v1.md §15`](../../../DroneFirmware/specification/architecture-contracts-v1.md).

---

## 7. Safety Model

Safety is enforced by `SafetyManager` in `src/safety/`.

**Arming preconditions (all required):**
- `AttitudeState.valid == true`
- No latched faults active
- All parameters within declared bounds

**Failsafe triggers:**
| Condition | Response |
|-----------|---------|
| Estimator invalid | Safe-state (motors inhibited) |
| Persistent `SchedulerMode::emergency` | System fault event emitted |
| AI command age > 25 ms | Automatic pilot fallback (not a full failsafe) |

Latched faults require explicit clear before re-arming. Full fault matrix in
[`specification/safety-fault-matrix-v1.md`](../../../DroneFirmware/specification/safety-fault-matrix-v1.md).

---

## 8. Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j$(nproc)
```

Requires CMake ≥ 3.20 and a C++17-capable compiler. Use `arm-none-eabi-g++` for hardware
targets and the system compiler for `SimHal`-based unit tests.

Every new `.cpp` added under `src/` must be listed in `target_sources(drone_firmware …)` in
`CMakeLists.txt`.

---

## 9. Contributing

- No heap allocation in fast-loop code (`new`, `malloc`, `std::vector` push without reserve).
- No blocking calls in fast-loop code (no `sleep`, no polling loops waiting on hardware).
- Every new scheduled task must declare `period_us` and `budget_us`.
- Every new binary struct on the telemetry/log path needs a `static_assert(sizeof(...) == N)`.
- New architectural decisions go in `specification/adr/` following the numbering convention.
- Requirement changes update `specification/verifiable-requirements-v1.md` and
  `specification/traceability-matrix-v1.md`.

---

Quick links: [Docs Index](README.md) | [Technical Concepts](technical-concepts.md) | [Glossary](glossary.md) | [Top](#droneos--architecture-overview)
