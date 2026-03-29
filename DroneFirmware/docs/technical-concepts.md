# DroneOS — Technical Concepts

Navigation: [Docs Index](README.md) | [Architecture](architecture.md) | [Technical Concepts](technical-concepts.md) | [Glossary](glossary.md) | [Specification Index](../../../DroneFirmware/specification/README.md) | [ADR Index](../../../DroneFirmware/specification/adr/README.md)

Status: Revision v1  
Date: 2026-03-29  
Scope: Explanation of all technical terms, patterns, and structures used in the project

---

## Table of Contents

1. [Wrench (Torque + Thrust)](#1-wrench-torque--thrust)
2. [ControlAllocator and Effectiveness Matrix](#2-controlallocator-and-effectiveness-matrix)
3. [AllocatorStatus](#3-allocatorstatus)
4. [Scheduler — Fixed-Time Executive](#4-scheduler--fixed-time-executive)
5. [SchedulerMode — Normal / Degraded / Emergency](#5-schedulermode--normal--degraded--emergency)
6. [Mixed-Criticality Scheduling](#6-mixed-criticality-scheduling)
7. [Major Cycle and Minor Slot](#7-major-cycle-and-minor-slot)
8. [WCET and Slot Budget](#8-wcet-and-slot-budget)
9. [Attitude Estimation — Quaternion + Gyro Bias](#9-attitude-estimation--quaternion--gyro-bias)
10. [ESKF — Error-State Kalman Filter](#10-eskf--error-state-kalman-filter)
11. [Innovation and Innovation Gate](#11-innovation-and-innovation-gate)
12. [Body Frame and Control Axes](#12-body-frame-and-control-axes)
13. [NED Frame](#13-ned-frame)
14. [AiControlLink and AiWrenchCommandV1](#14-aicontrollink-and-aiwrenchcommandv1)
15. [AiStateSnapshotV1](#15-aistatesnapshotv1)
16. [Command Freshness / Staleness](#16-command-freshness--staleness)
17. [Hardware Graph Engine](#17-hardware-graph-engine)
18. [DroneCAN](#18-dronecan)
19. [GraphFingerprint and Component Identity](#19-graphfingerprint-and-component-identity)
20. [Physics Model Engine](#20-physics-model-engine)
21. [Snapshot and Zero-Copy](#21-snapshot-and-zero-copy)
22. [DMA-First I/O Kernel](#22-dma-first-io-kernel)
23. [ContractHeaderV1 and Schema Versioning](#23-contractheaderv1-and-schema-versioning)
24. [Parameters — ParameterRecordV1](#24-parameters--parameterrecordv1)
25. [Events — EventRecordV1](#25-events--eventrecordv1)
26. [Diagnostics — DiagnosticRecordV1](#26-diagnostics--diagnosticrecordv1)
27. [Measurement — MeasurementRecordV1](#27-measurement--measurementrecordv1)
28. [Binary Telemetry and SD Log](#28-binary-telemetry-and-sd-log)
29. [Arming and Failsafe](#29-arming-and-failsafe)
30. [Rate Loop and Bandwidth](#30-rate-loop-and-bandwidth)
31. [Anti-Windup](#31-anti-windup)
32. [DroneFactory](#32-dronefactory)

---

## 1. Wrench (Torque + Thrust)

**What it is:**  
"Wrench" is the mechanics term that describes the combination of a torque (moment) vector and a force (thrust) applied to a rigid body. In DroneOS, the external control interface is defined in wrench space, not in angles or position.

**Why it matters:**  
Traditional autopilots (ArduPilot, PX4) receive angle or rate setpoints. DroneOS exposes torque directly, eliminating the conversion layer and allowing AI agents to express precise physical intent.

**In code:**
```cpp
// ControlDemand in src/control/ControlLoop.hpp
struct ControlDemand {
    float roll_rate_demand;   // indirect torque via rate loop
    float pitch_rate_demand;
    float yaw_rate_demand;
    float thrust_demand;      // normalized collective thrust
};
```

The AI interface uses `AiWrenchCommandV1`, which carries `tau[3]` (torques on roll, pitch, yaw axes in Nm) and `thrust` (N).

---

## 2. ControlAllocator and Effectiveness Matrix

**What it is:**  
The `ControlAllocator` receives the desired wrench `[τx, τy, τz, Fz]` and computes the individual command for each motor. It does so using an **effectiveness matrix** — a matrix that describes how each motor contributes to each torque axis and to the total thrust.

**Why it matters:**  
A static mixer (fixed Quad-X) cannot handle asymmetric geometries, failed actuators, or custom frames. The ControlAllocator solves this by decoupling geometry from the algorithm.

**Quad-X — typical per-motor contribution:**

| Motor | Roll (τx) | Pitch (τy) | Yaw (τz) | Thrust |
|-------|-----------|------------|---------|--------|
| M1 FL | −         | +          | +       | +      |
| M2 FR | +         | +          | −       | +      |
| M3 RL | −         | −          | −       | +      |
| M4 RR | +         | −          | +       | +      |

**Desaturation (bounded sequential scaling):**  
When a motor hits its physical limit (0–1), the algorithm iteratively recalculates the contributions of the remaining motors to satisfy the wrench as closely as possible, without violating actuation limits.

**In code:** `src/control/ControlAllocator.hpp` / `.cpp`

---

## 3. AllocatorStatus

**What it is:**  
A structure produced every cycle by the `ControlAllocator` describing the allocation result.

```cpp
struct AllocatorStatus {
    float    unallocated_tau[3]; // unachieved torque (Nm)
    float    unallocated_thrust; // unachieved thrust (N)
    uint32_t saturated_mask;     // bit per motor: 1 = saturated
    uint32_t failed_mask;        // bit per motor: 1 = excluded
    uint32_t condition_warning;  // 1 = ill-conditioned matrix
    uint32_t solve_iterations;   // iterations used
};
```

**Why it matters:**  
`unallocated_tau` is the unachievable torque residual. The control loop uses this to implement **allocator-aware anti-windup** — preventing the PID integrator from accumulating error for torque the motors cannot physically produce.

---

## 4. Scheduler — Fixed-Time Executive

**What it is:**  
The DroneOS `Scheduler` is a **time-triggered cooperative fixed-priority executive**. It does not use an RTOS. Each task declares a period and a WCET budget.

**Execution order per cycle:**

```
1. estimation_fast    (AttitudeEstimator, SensorLatching)
2. critical_fast      (ControlLoop, ControlAllocator, SafetyCheck, OutputCommit)
3. mission_normal     (FlightMode, Navigation)
4. service_background (Telemetry, Logging, ParameterService)
```

This order guarantees that the attitude estimate is up to date **before** the control loop runs — a bug present in the original version where `critical_fast` ran before `estimation_fast`.

**In code:** `src/runtime/Scheduler.hpp` / `.cpp`

---

## 5. SchedulerMode — Normal / Degraded / Emergency

**What it is:**  
The scheduler monitors budget violations across cycles and transitions between three modes:

| Mode        | Entry condition                        | Effect                                              |
|-------------|----------------------------------------|-----------------------------------------------------|
| `normal`    | default                                | all tasks execute                                   |
| `degraded`  | ≥ 3 consecutive violations             | `service_background` is skipped                    |
| `emergency` | ≥ 10 consecutive violations            | `mission_normal` + `service_background` are skipped |

Recovery: 200 consecutive clean cycles return to the previous mode.

**Why it matters:**  
Ensures that temporary overload never compromises high-criticality functions (estimation + control).

---

## 6. Mixed-Criticality Scheduling

**What it is:**  
A scheduling model that assigns a **criticality level** to each task. HI-criticality tasks have two budgets: a LO budget (normal operation) and a HI budget (emergency). When the system detects overload, LO tasks are dropped to guarantee HI tasks continue.

**In DroneOS:**

| Class                | Criticality | Can be skipped?   |
|----------------------|-------------|-------------------|
| `estimation_fast`    | HI          | Never             |
| `critical_fast`      | HI          | Never             |
| `mission_normal`     | LO          | In emergency      |
| `service_background` | LO          | In degraded+      |

---

## 7. Major Cycle and Minor Slot

**Major Cycle:**  
The fixed period of the deterministic loop. DroneOS v1 target: **2.0 ms at 500 Hz**.

**Minor Slot:**  
A fixed temporal subdivision within the major cycle, assigned to a specific task. Examples:

- Sensor latch slot (IMU read via DMA)
- Estimation slot
- Control slot
- Allocation slot
- Output commit slot

The sum of all slot WCETs must be `≤ 2.0 ms`.

**In code:** the main loop in `src/app/main.cpp` applies a residual `sleep` to fill exactly 2 ms:
```cpp
constexpr common::DurationUs k_major_cycle_us = 2000;
// ... after run_once():
hal.sleep_us(residual);
```

---

## 8. WCET and Slot Budget

**WCET (Worst-Case Execution Time):**  
The declared upper bound on a task's execution time. It is a contract — not an average.

**Slot Budget:**  
The WCET allocated to a specific slot. Every task declares its budget when registered with the scheduler:

```cpp
scheduler.add_task("control_update", PriorityClass::critical_fast,
                   /*period_us=*/2000, /*budget_us=*/400, ...);
```

**Overrun:** execution beyond the declared budget. The scheduler tracks `overrun_count` per task and uses it to compute cycle violations.

---

## 9. Attitude Estimation — Quaternion + Gyro Bias

**What it is:**  
The `AttitudeEstimator` maintains the vehicle's orientation estimate in 3D space. The original version used **Euler integration** with a simple complementary filter (prone to gimbal lock and drift). The current version uses **quaternion** propagation.

**Quaternion:**  
A 4-dimensional rotation representation `q = [w, x, y, z]` that avoids singularities (gimbal lock) and is computationally efficient for gyroscopic rate integration.

**Angular rate integration:**

$$
\dot{q} = \frac{1}{2} q \otimes \begin{bmatrix} 0 \\ \omega_x \\ \omega_y \\ \omega_z \end{bmatrix}
$$

Applied as:

$$
q_{k+1} \approx q_k + \dot{q} \cdot \Delta t
$$

followed by renormalization to `|q| = 1`.

**Gyro Bias:**  
MEMS gyroscopes have a slow offset that drifts with temperature. The estimator maintains `gyro_bias_rad_s[3]` and subtracts it before integration:

```cpp
float omega_corrected[i] = omega_raw[i] - gyro_bias_rad_s[i];
```

The bias is incrementally adapted at every accelerometer correction (gain 0.02).

**Backward-compatible output:** the estimator converts the final quaternion to Euler angles `(roll, pitch, yaw)` to avoid breaking the rest of the pipeline.

---

## 10. ESKF — Error-State Kalman Filter

**What it is:**  
The ESKF (Error-State Kalman Filter, also called Indirect Kalman Filter) is the standard approach for navigation estimation in embedded systems.

Instead of estimating the absolute state (position, velocity, quaternion) with a full Kalman filter, the ESKF estimates the **error** over a nominally propagated state. The nominal state is propagated by IMU integration and the Kalman correction is applied only when an external observation (GNSS, baro, magnetometer) arrives.

**Advantages over the complementary filter:**

| Aspect               | Complementary Filter      | ESKF                            |
|----------------------|---------------------------|---------------------------------|
| Representation       | Euler or simple quaternion| Quaternion + covariance         |
| Sensor fusion        | Fixed gain                | Optimal gain (Jacobian)         |
| Bias estimation      | Simple                    | Estimated with covariance       |
| Scalability          | Limited                   | GPS, baro, mag, vision          |

**Current project status:**  
The current implementation (`AttitudeEstimator`) is quaternion + bias without the full covariance matrix. It is an intermediate step toward the full ESKF planned in future iterations.

---

## 11. Innovation and Innovation Gate

**Innovation:**  
The difference between a sensor's actual observation and the model's prediction:

$$
\tilde{y} = z - h(\hat{x})
$$

Where `z` is the measurement (e.g., the gravity vector from the accelerometer) and `h(x̂)` is the prediction based on the current quaternion.

**Innovation Gate:**  
Rejects an observation whose innovation exceeds a threshold. Prevents spurious spikes (mechanical vibration, impacts) from corrupting the estimator.

**In code:**
```cpp
// gate: 0.8 rad
if (innovation_norm < 0.8f) {
    // apply accelerometer correction
    state_.innovation_gated = false;
} else {
    state_.innovation_gated = true; // measurement rejected
}
```

`innovation_gated = true` is signalled in `AiStateSnapshotV1` so the AI agent knows the estimator is running on gyroscope only for that cycle.

---

## 12. Body Frame and Control Axes

**What it is:**  
The **body frame** is the reference system fixed to the vehicle. The axes are:

- **X (roll):** points toward the front of the vehicle
- **Y (pitch):** points to the right of the vehicle
- **Z (yaw / thrust):** points downward (NED convention) or upward (ROS/body-up convention)

DroneOS uses **+Z body = upward** for thrust (body-up convention).

**Torques in the body frame:**

| Axis  | Variable      | Visual effect      |
|-------|---------------|--------------------|  
| τx    | `tau[0]`      | Roll               |
| τy    | `tau[1]`      | Pitch              |
| τz    | `tau[2]`      | Yaw                |

---

## 13. NED Frame

**What it is:**  
**N**orth-**E**ast-**D**own — the standard global reference system in aviation.

- **X:** points to geographic North
- **Y:** points to the East
- **Z:** points downward (toward the center of the Earth)

Velocity (`velocity_ned`) and position (`position_ned`) are expressed in NED in `AiStateSnapshotV1`. When GNSS is unavailable, these fields are zero.

---

## 14. AiControlLink and AiWrenchCommandV1

**What it is:**  
`AiControlLink` is the ingress channel for commands from an external AI agent into the Deterministic Core. It is the implementation of the `AiWrenchCommandV1` contract.

**Command structure:**

```cpp
struct AiWrenchCommandV1 {
    uint16_t version;      // must be 1
    uint16_t flags;        // bit1 = emergency_stop
    uint32_t seq;          // monotonic sequence number
    TimestampUs t_cmd_us;  // monotonic timestamp when the AI generated the command
    float tau[3];          // torques [roll, pitch, yaw] in Nm
    float thrust;          // collective thrust in N
};
```

**Validations applied in `ingest_wrench()`:**

1. `version == 1` — rejects incompatible versions
2. `t_cmd_us <= now_fcu_us` — rejects future-dated commands
3. `age <= max_latency_us` — rejects commands that are too old (default: 25 ms)
4. `seq > last_seq_` — rejects out-of-order or replayed commands

**Pilot fallback:**  
If no fresh AI command is available, the system uses the pilot flight mode. The selection is made in `control_update_task`:

```cpp
if (app->ai_control_link->has_fresh_command(now_us, k_ai_command_max_latency_us)) {
    // use AI wrench
} else {
    // use pilot flight mode (rate/attitude)
}
```

**In code:** `src/comms/AiControlLink.hpp` / `.cpp`

---

## 15. AiStateSnapshotV1

**What it is:**  
The egress channel: the drone's complete "sensorium" sent to the AI agent every deterministic cycle. The AI receives everything it needs to close the control loop.

**Main fields:**

| Field                   | Description                                             |
|-------------------------|---------------------------------------------------------|
| `q[4]`                  | Attitude quaternion `[w, x, y, z]`                     |
| `omega_body[3]`         | Angular rates in body frame (rad/s)                    |
| `accel_body[3]`         | Linear acceleration in body frame (m/s²)              |
| `gyro_bias[3]`          | Estimated gyroscope bias (rad/s)                       |
| `velocity_ned[3]`       | NED velocity (m/s) — zero without GNSS                |
| `position_ned[3]`       | NED position (m) — zero without GNSS                  |
| `innovation_norm[3]`    | Innovation residual norm per sensor class              |
| `gate_status`           | Bit per sensor: 1 = within gate                        |
| `scheduler_mode`        | 0=NORMAL, 1=DEGRADED, 2=EMERGENCY                      |
| `alloc_saturated_mask`  | Bit per actuator: 1 = saturated this cycle             |
| `alloc_unallocated[4]`  | Unachieved wrench residual `[τx, τy, τz, Fz]`         |
| `safety_state`          | Bitmask of active safety faults                        |
| `arming_state`          | 0=DISARMED, 1=ARMED                                    |

**Why it matters:**  
An AI agent receiving this snapshot per cycle has complete visibility of physical state, estimator health, actuator saturation, and scheduler mode — without needing any other data source.

---

## 16. Command Freshness / Staleness

**What it is:**  
A **stale command** is one whose generation timestamp (`t_cmd_us`) differs from the current FCU time by more than the configured threshold.

**Why it matters:**  
AI inference can be delayed. If the FC keeps executing a command generated 200 ms ago based on an already-outdated world view, the result can be dangerous.

**Default threshold:** `k_ai_command_max_latency_us = 25 000 µs` (25 ms).

When the command expires, `has_fresh_command()` returns `false` and the system automatically falls back to pilot mode.

---

## 17. Hardware Graph Engine

**What it is:**  
The subsystem that discovers, models, and tracks all hardware components connected to the drone. It builds a **runtime hardware graph**.

**Graph structure:**

- **Nodes:** each component is a `NodeDescriptor` with type, model, serial hash, capabilities, and state
- **Edges:** bus and hierarchy relationships between components
- **Fixed capacity:** maximum 64 nodes, maximum 128 edges — no dynamic allocation

**Modeled components:**

| Type              | Examples                            |
|-------------------|-------------------------------------|
| `COMPONENT_IMU`   | ICM-42688-P, BMI088                 |
| `COMPONENT_ESC`   | BLHeli32, VESC                      |
| `COMPONENT_MOTOR` | brushless motors                    |
| `COMPONENT_BARO`  | MS5611, BMP388                      |
| `COMPONENT_GNSS`  | u-blox M9N, F9P                     |
| `COMPONENT_BATTERY` | LiPo pack                         |
| `COMPONENT_COMPUTE` | companion computer                |

**Used by ControlAllocator:**  
Actuator geometry (position, orientation, motor model) is read from the Hardware Graph to dynamically generate the **effectiveness matrix**.

---

## 18. DroneCAN

**What it is:**  
DroneCAN (formerly UAVCAN v0) is a CAN bus protocol for drone systems. It provides:

- **Automatic discovery:** each CAN node announces its Node ID and responds to `GetNodeInfo`
- **Hardware UID:** 128 unique bits per device, used for identity hashing
- **Standardized messages:** ESC commands, IMU data, battery status, GNSS fixes
- **Fault tolerance:** priority-based arbitration, CRC error detection

**In DroneOS:**  
DroneCAN is the **primary** hardware discovery channel (ADR-0008). I2C/SPI are secondary channels for sensors that do not support CAN.

---

## 19. GraphFingerprint and Component Identity

**GraphFingerprint:**  
A CRC32 computed over the ordered set of `serial_hash32` values of all active components. It acts as a "fingerprint" of the current hardware configuration.

**Usage:**  
- Stored at manufacturing/commissioning as the baseline
- Compared at each boot or after maintenance
- If the fingerprint changes → a component was replaced → **Conditional Revalidation** is triggered

**Component Identity:**  
Each component has a `serial_hash32` = CRC32 over a normalized fingerprint blob (model + serial number + known calibration). This detects part replacement even when the component type is identical.

---

## 20. Physics Model Engine

**What it is:**  
The subsystem that translates the Hardware Graph into physical vehicle parameters:

- Initial control gains and estimated loop bandwidth
- Per-motor current and thrust limits
- Safety envelope (attitude, angular rate, and acceleration limits)
- Approximate dynamic model for the allocator (estimated inertia)

**Inputs:** motor constants (KV, resistance), propeller geometry, battery model, frame mass and inertia.

**Outputs:** automatically configure the `ControlAllocator` and safety parameters without manual intervention.

---

## 21. Snapshot and Zero-Copy

**Snapshot:**  
An immutable, consistent view of system state produced at each cycle boundary. Consumers always read a complete snapshot — never a partially updated state.

**Double-buffering:**  
The core maintains two snapshot buffers. The producer writes to buffer B while consumers read from buffer A. At the end of the cycle, pointers are swapped atomically (`tick_id` changes).

**Zero-Copy:**  
A data transfer pattern in which a pointer or reference is passed instead of copying the data. In the fast loop, avoiding unnecessary copies is critical to staying within the WCET.

---

## 22. DMA-First I/O Kernel

**What it is:**  
DMA (Direct Memory Access) is a hardware mechanism that transfers data between peripherals and memory without CPU involvement. In DroneOS, all I/O operations on the critical path are **DMA-first**, following the policy:

- **enqueue-or-fail:** the operation is enqueued if DMA is available, or discarded with a log entry if not — **never blocks**
- The CPU never busy-waits for sensor data or write acknowledgements

**Why it matters:**  
In a 2 ms loop, a single polling-based sensor read (even a fast one) can consume tens of microseconds of bus latency, compromising the WCET.

**Flow:**

```
1. DMA starts the SPI/I2C transfer from the IMU at the beginning of the cycle
2. control loop executes with data from the previous cycle (already in RAM)
3. on the next cycle, DMA has already completed and new data is available
```

---

## 23. ContractHeaderV1 and Schema Versioning

**What it is:**  
All records persisted or transmitted by DroneOS carry a fixed 28-byte binary header:

```c
typedef struct {
    uint16_t schema_version; // must be DRONEOS_SCHEMA_V1 = 1
    uint16_t entity_kind;    // record type (param, event, etc.)
    uint32_t record_size;    // total size in bytes
    uint32_t id32;           // stable entity identifier
    uint64_t time_us;        // monotonic timestamp
    uint32_t tick_id;        // deterministic cycle index
    uint32_t crc32;          // CRC32 over the entire record
} ContractHeaderV1;
```

**Schema Versioning:**  
- Compatible changes (appending fields at the end of non-critical records) do not increment `schema_version`
- Incompatible changes increment `schema_version` and require a migration period with dual-reader support
- All readers validate `schema_version` before using the content

**Mandatory validation order:**
1. header bounds and `record_size`
2. `schema_version` support
3. `entity_kind` recognized
4. CRC32 verification
5. entity-specific invariants

---

## 24. Parameters — ParameterRecordV1

**What it is:**  
Tunable configuration values with direct indexing and enforced bounds:

- `index`: O(1) access into the parameter table
- `min_f32` / `max_f32`: declared bounds; every write is validated
- `generation`: counter that increments on each accepted write — detects stale reads
- `flags.bit0 = persistent`: marks the value as surviving reboot
- `flags.bit2 = safety_locked`: parameter cannot be modified while armed

**Parameter examples:**

| Name                        | Type  | Description                             |
|-----------------------------|-------|-----------------------------------------|
| `RATE_ROLL_KP`              | F32   | Proportional gain of the rate loop      |
| `RATE_PITCH_KP`             | F32   | Proportional gain pitch                 |
| `THRUST_MAX_N`              | F32   | Maximum thrust for normalization        |
| `SCHED_DEGRADED_THRESHOLD`  | U32   | Violations before entering degraded     |

---

## 25. Events — EventRecordV1

**What it is:**  
Records of discrete occurrences with significance for safety policy:

```c
typedef enum { EVENT_INFO, EVENT_WARNING, EVENT_ERROR, EVENT_CRITICAL } EventSeverity;
typedef enum { EVENT_SRC_SCHEDULER, EVENT_SRC_SAFETY, EVENT_SRC_CONTROL,
               EVENT_SRC_ESTIMATOR, EVENT_SRC_HW_GRAPH, ... } EventSource;
```

**Latched Events:**  
Safety events with `latch = 1` persist even after the condition clears. They require an explicit clear — ensuring crew or maintenance systems are notified.

**Event Code Namespace:**  
`bits[31:24]` = source domain, `bits[23:16]` = category within domain, `bits[15:0]` = specific event. Stable across patch releases.

---

## 26. Diagnostics — DiagnosticRecordV1

**What it is:**  
Health, budget, and fault-containment status for runtime observability:

| Kind           | Example                                         |
|----------------|---------------------------------------------------|
| `DIAG_BUDGET`  | `scheduler.control_task: 380µs / 400µs budget`  |
| `DIAG_WCET`    | `estimator: max observed 310µs`                 |
| `DIAG_FAULT`   | `IMU: single-sample gap detected`                |
| `DIAG_HEALTH`  | `battery: 20% remaining`                         |

Every `DIAG_STATUS_FAILED` on a safety-critical component must simultaneously emit `EVENT_CRITICAL`.

---

## 27. Measurement — MeasurementRecordV1

**What it is:**  
Timestamped observed values with quality and domain:

```c
typedef enum { MEAS_QUALITY_OK, MEAS_QUALITY_SATURATED,
               MEAS_QUALITY_STALE, MEAS_QUALITY_INVALID } MeasurementQuality;
```

**STALE quality:**  
Emitted when data is older than `staleness_threshold_us`. The consumer uses the last valid sample and notifies via an Event.

**Tick consistency:**  
All Measurements produced in the same cycle share the same `tick_id`, ensuring consistency when correlating data from different sources.

---

## 28. Binary Telemetry and SD Log

**TelemetryFrame:**  
Fixed-size binary frame transmitted over UART every cycle. Contains:
- Attitude (roll, pitch, yaw)
- Angular rates
- Control demand
- Motor outputs
- `allocator_status` (saturated_mask, condition_warning, solve_iterations)
- `scheduler_mode` (normal/degraded/emergency)

The frame size is verified by `static_assert` at compile time — no silent layout changes are possible.

**BinaryLogRecord:**  
Written to the SD card with the same content as the TelemetryFrame. The backpressure policy is **enqueue-or-discard** — the fast loop never blocks waiting for SD.

---

## 29. Arming and Failsafe

**Arming:**  
An explicit transition that enables actuation output. Requires all **arming preconditions** to be satisfied:
- estimator valid
- safety parameters within bounds
- no active latched faults
- Hardware Graph fingerprint matching the baseline

**Failsafe:**  
A protective transition triggered by a critical condition:

| Condition                        | Action                              |
|----------------------------------|-------------------------------------|
| AI command stale > 25 ms         | Fallback to pilot mode              |
| RC link loss                     | Signal-loss mode                    |
| Persistent critical overrun      | SchedulerMode::emergency            |
| IMU invalid                      | Safe-state (motors off)             |

**Latched Failsafe:** persists after the cause resolves. Requires disarming and explicit clear.

---

## 30. Rate Loop and Bandwidth

**Rate Loop:**  
The **inner** control loop that regulates the vehicle's angular rate (`p, q, r`). It is the highest-frequency loop (500 Hz in DroneOS v1).

```
rate setpoint → rate error → PID → wrench → allocator → motors
```

**Bandwidth:**  
The frequency range within which the control loop provides useful corrective response. Depends on:
- PID gains
- Total delay (sensor latency + compute latency + actuator latency)
- Vehicle inertia

Low inertia + high gains + fast loop = high bandwidth = more agile drone.

---

## 31. Anti-Windup

**What it is:**  
A mechanism that prevents unlimited accumulation of the PID integrator when actuators are saturated.

**Without anti-windup:** when the drone cannot execute the requested torque (motors saturated), the error persists. The integrator accumulates indefinitely. When the drone leaves saturation, the integrator discharges explosively.

**In DroneOS:**  
Anti-windup is **allocator-aware**: it uses `AllocatorStatus.unallocated_tau` to know exactly which torque component was unachievable, and zeroes only that portion of the integrator — instead of a generic clamp-based limit.

---

## 32. DroneFactory

**What it is:**  
The DroneOS cognitive manufacturing and validation subsystem. Responsible for:

1. **Automatic hardware recognition:** detects components via Hardware Graph on the assembly line
2. **Compatibility validation:** verifies that the component set is compatible and certified
3. **Automated calibration:** executes calibration sequences (IMU, ESC, magnetometer)
4. **Physics Model Engine:** generates initial vehicle parameters based on discovered hardware
5. **Component Hash Snapshot:** records the fingerprint baseline for future maintenance detection

**Temporal isolation:**  
DroneFactory operates **outside** the Deterministic Core. All its execution is in the low-criticality domain (idle windows or ground context). It never accesses the fast loop directly.

---

## References

| Document                           | Location                                                                       |
|------------------------------------|--------------------------------------------------------------------------------|
| Architecture v1                    | [specification/architecture-v1.md](../../../DroneFirmware/specification/architecture-v1.md) |
| Architecture Contracts v1          | [specification/architecture-contracts-v1.md](../../../DroneFirmware/specification/architecture-contracts-v1.md) |
| Glossary v1                        | [specification/glossary-v1.md](../../../DroneFirmware/specification/glossary-v1.md) |
| Strategy v1                        | [specification/strategy-v1.md](../../../DroneFirmware/specification/strategy-v1.md) |
| ADR-0005 ESKF Estimator            | [specification/adr/0005-eskf-estimator-foundation.md](../../../DroneFirmware/specification/adr/0005-eskf-estimator-foundation.md) |
| ADR-0006 ControlAllocator          | [specification/adr/0006-control-allocator-over-mixer.md](../../../DroneFirmware/specification/adr/0006-control-allocator-over-mixer.md) |
| ADR-0007 DMA-First I/O             | [specification/adr/0007-dma-first-io-kernel.md](../../../DroneFirmware/specification/adr/0007-dma-first-io-kernel.md) |
| ADR-0008 DroneCAN Hardware Discovery | [specification/adr/0008-droneCAN-first-hardware-discovery.md](../../../DroneFirmware/specification/adr/0008-droneCAN-first-hardware-discovery.md) |

---

Owner: DroneOS Architecture  
Related: `src/` — implementation of the concepts documented here

---

Quick links: [Docs Index](README.md) | [Architecture](architecture.md) | [Glossary](glossary.md) | [Top](#droneos--technical-concepts)
