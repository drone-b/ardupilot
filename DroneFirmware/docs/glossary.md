# DroneOS — Domain Glossary

Navigation: [Docs Index](README.md) | [Architecture](architecture.md) | [Technical Concepts](technical-concepts.md) | [Glossary](glossary.md) | [Specification Glossary](../../../DroneFirmware/specification/glossary-v1.md) | [ADR Index](../../../DroneFirmware/specification/adr/README.md)

Status: Reference  
Updated: 2026-03-29

This is the unified reference glossary for all domain terms used across the DroneOS project.
It covers vehicle physics, control theory, estimation, sensors, realtime systems, communications,
and flight behavior concepts.

For DroneOS-specific architectural terms (Major Cycle, SchedulerMode, AiControlLink, etc.) see
[technical-concepts.md](technical-concepts.md).  
For the canonical specification-level glossary see
[`specification/glossary-v1.md`](../../../DroneFirmware/specification/glossary-v1.md).

---

## 1. Vehicle and Frame Terms

| Term | Meaning |
|------|---------|
| Airframe | Physical vehicle type or geometry: quadcopter, hexacopter, plane, rover. |
| Multirotor | Rotorcraft with multiple propellers providing lift and directional control. |
| VTOL | Vertical Takeoff and Landing vehicle. |
| Frame | Motor/servo geometry defining how motors are distributed around the vehicle center. |
| X configuration | Quadcopter layout with arms at 45° to the forward axis. |
| Plus configuration | Quadcopter layout where one arm aligns with the forward axis. |
| Body frame | Coordinate system fixed to the vehicle (X=forward, Y=right, Z=up in DroneOS convention). |
| NED frame | North-East-Down global reference frame used for navigation and position data. |
| Earth frame | External reference frame used for navigation; often synonymous with NED for ground vehicles. |
| Home | Reference position used for return logic and relative navigation. |
| Geofence | Spatial operating boundary that may constrain or trigger safety behavior. |
| Terrain following | Altitude or clearance control relative to ground shape rather than absolute altitude. |

---

## 2. Attitude, Motion, and Control Terms

| Term | Meaning |
|------|---------|
| Attitude | Vehicle orientation in 3D space. |
| Roll | Rotation around the longitudinal (X) body axis. |
| Pitch | Rotation around the lateral (Y) body axis. |
| Yaw | Rotation around the vertical (Z) body axis. |
| Angular rate | Rotational speed around a body axis (rad/s). Usually labeled p, q, r. |
| Wrench | Combined torque vector and thrust scalar representing the external control demand. |
| Torque | Rotational effect applied about a body axis (Nm). |
| Thrust | Collective force generated along the +Z body axis (N). |
| Rate loop | Inner control loop that regulates angular rates. Runs at the highest frequency. |
| Attitude loop | Outer control loop that drives the attitude toward a target by generating rate demands. |
| Position loop | Control loop that regulates vehicle position and velocity. |
| Cascaded control | Nested loop structure where outer loops generate setpoints for inner loops. |
| Setpoint / target | Desired reference value for a controller. |
| Error | Difference between the target and the measured state. |
| PID | Proportional-Integral-Derivative controller topology. |
| P term | Immediate response proportional to the current error. |
| I term | Accumulated response that eliminates steady-state bias. |
| D term | Predictive damping based on rate of change. |
| Anti-windup | Logic that prevents unlimited integrator accumulation when actuators are saturated. |
| Saturation | Condition where a controller output or motor command hits its physical limit. |
| Derivative on measurement | Derivative computed from the measured signal rather than error, eliminating setpoint kick. |
| Feedforward | Controller component driven by the commanded motion rather than feedback error. |
| Slew rate | Maximum rate of change allowed for a command or output. |
| Autotune | Assisted or automated process for finding optimal control gains. |
| Bandwidth | Frequency range over which a control loop provides useful corrective response. |
| Thrust-to-weight | Ratio of maximum total thrust to vehicle weight, a measure of agility. |

---

## 3. Actuation and Propulsion Terms

| Term | Meaning |
|------|---------|
| Motor output | Normalized command sent to a brushless motor channel [0..1]. |
| Servo output | Command sent to a servo actuator, typically for control surfaces. |
| ESC | Electronic Speed Controller — converts normalized commands to motor drive signals. |
| PWM | Pulse-width modulation — analog encoding of motor commands. |
| DShot | Digital ESC protocol with no analog timing ambiguity. |
| Effectiveness matrix | Matrix mapping body wrench demands to individual motor commands. |
| Control allocator | Algorithm solving the effectiveness-matrix inversion under saturation constraints. |
| Desaturation | Iterative rescaling of motor commands when one or more motors hit their limit. |
| Allocation residual | Unachieved portion of the commanded wrench after desaturation. |
| Motor loss handling | Logic reducing control authority claims when a motor is declared failed. |
| Spin arm speed | Minimum motor speed maintained when armed but not commanding flight thrust. |

---

## 4. State Estimation and Navigation Terms

| Term | Meaning |
|------|---------|
| State estimator | Software that fuses sensor measurements into attitude, velocity, position, and bias estimates. |
| AHRS | Attitude and Heading Reference System. |
| EKF | Extended Kalman Filter — nonlinear optimal estimator. |
| ESKF | Error-State Kalman Filter — Kalman filter operating on the error state around a nominal trajectory. |
| Complementary filter | Lightweight method blending fast gyro integration with slower absolute corrections. |
| Quaternion | 4-component rotation representation `[w, x, y, z]` that avoids gimbal lock. |
| Gimbal lock | Singularity in Euler angle representation that loses one degree of freedom. |
| Sensor fusion | Combining multiple sensor inputs into a single, more robust estimate. |
| Innovation | Residual between a filter's predicted measurement and the actual measurement. |
| Innovation gate | Threshold beyond which an observation is rejected as an outlier. |
| Propagation | Estimator step that advances the state using IMU integration without an external correction. |
| Correction | Estimator step that adjusts the state toward a measurement from an external reference. |
| Covariance | Filter quantity representing state uncertainty and cross-correlations. |
| Gyro bias | Slowly drifting offset in the angular-rate sensor (rad/s). |
| Bias adaptation | Incremental correction of estimated gyro or accelerometer bias. |
| Drift | Accumulating error from imperfect inertial integration or uncompensated bias. |
| Dead reckoning | Continuing motion estimation after loss of external references using inertial propagation only. |
| Source switching | Transitioning between measurement sources while maintaining state continuity. |
| Lane switching | Selecting between multiple parallel estimator instances. |

---

## 5. Sensor Terms

| Term | Meaning |
|------|---------|
| IMU | Inertial Measurement Unit — gyroscope + accelerometer assembly. |
| INS | Inertial Navigation System. |
| Gyroscope | Sensor measuring angular rate. |
| Accelerometer | Sensor measuring specific force (acceleration including gravity). |
| Magnetometer | Sensor measuring magnetic field, used for heading reference. |
| Barometer | Pressure sensor used for altitude estimation. |
| GNSS / GPS | Global Navigation Satellite System, providing absolute position and velocity. |
| Optical flow | Sensor measuring apparent ground motion from imagery for velocity aiding. |
| Rangefinder / LiDAR | Sensor measuring distance to a surface. |
| Delta angle | Integral of angular rate over one sample interval. |
| Delta velocity | Integral of specific force over one sample interval. |
| Sample period | Fixed time between sensor measurements. |
| Sample age | Elapsed time between sample generation and use. |
| Transport latency | Delay between physical measurement and software availability. |
| Data-ready interrupt | Hardware signal indicating a sensor sample is ready for read. |
| Vibration | Mechanical noise corrupting inertial sensor quality. |
| Harmonic notch filter | Narrowband filter tracking motor/rotor harmonics to suppress vibration. |
| Health flag | Indicator that a sensor output is within valid operating bounds. |
| Glitch | Short, anomalous measurement disturbance. |
| Calibration | Estimating and correcting sensor offsets, scales, and axis misalignments. |
| Temperature compensation | Correcting sensor drift caused by temperature change. |

---

## 6. Safety, Arming, and Fault Terms

| Term | Meaning |
|------|---------|
| Arming | Explicit transition that authorizes propulsion and control output. |
| Disarm | Transition that removes propulsion authority. |
| Arming precondition | Condition that must be satisfied before arming is allowed. |
| Pre-arm check | Validation sequence run before arming is permitted. |
| Failsafe | Protective state transition triggered by a safety-critical fault or loss of required input. |
| Latched failsafe | A failsafe that persists after the triggering condition clears, requiring explicit clear. |
| Land detector | Logic estimating whether the vehicle is on the ground. |
| Crash check | Logic for detecting probable loss of control or impact. |
| Emergency stop | Immediate, unconditional propulsion shutdown path. |
| Watchdog | Hardware/software mechanism detecting stalled or hung execution. |
| Health monitor | Supervisory logic observing whether subsystems remain within valid operating bounds. |
| Stale command | A command whose timestamp age exceeds the declared freshness threshold. |
| Safety envelope | Set of bounds on attitude, rate, and acceleration defining the safe operating region. |

---

## 7. Flight Mode Terms

| Term | Meaning |
|------|---------|
| Flight mode | Top-level behavioral policy that defines input interpretation, target generation, and autonomy level. |
| Stabilize | Pilot commands attitude, stabilization loop maintains roll/pitch. |
| Acro | Pilot commands angular rates directly; full manual authority. |
| AltHold | Altitude-hold mode; vertical motion stabilized, horizontal manual. |
| Loiter / PosHold | Position-hold mode; GNSS-based horizontal stability. |
| RTL | Return-to-launch or return-to-home behavior. |
| Auto | Fully mission-driven behavior executing stored waypoints and commands. |
| Guided | Externally directed mode accepting targets from GCS or companion computer. |
| Rate command | Pilot input mapped to a target angular rate. |
| Angle command | Pilot input mapped to a target roll/pitch angle. |
| Heading hold | Automatic yaw bias maintaining last commanded heading when no yaw input is present. |
| Mode manager | Logic responsible for selecting, validating, and enforcing the active flight mode. |

---

## 8. Realtime and Embedded Systems Terms

| Term | Meaning |
|------|---------|
| Determinism | Property that execution order, timing, and output are bounded and predictable for the same input. |
| WCET | Worst-Case Execution Time — declared upper bound on a task's runtime. |
| Major cycle | Fixed period of the deterministic loop. DroneOS v1: 2.0 ms at 500 Hz. |
| Minor slot | Temporal subdivision within the major cycle assigned to a specific task stage. |
| Task budget | Time allocation declared for one invocation of a scheduled task. |
| Overrun | Task execution that exceeds its declared time budget. |
| Slack | Time remaining before the next higher-priority deadline becomes due. |
| Deadline miss | Failure to complete work before its release or completion deadline. |
| Backlog catch-up | Running delayed tasks in rapid succession — explicitly forbidden for flight-critical tasks. |
| Mixed-criticality | Scheduling model assigning LO/HI criticality levels with dual budgets. |
| Cooperative scheduling | Scheduling model where tasks run to completion; no forced preemption. |
| Fixed-priority scheduling | Static ordering of tasks by declared importance level. |
| Release time | Absolute time at which a periodic task becomes eligible to run. |
| Event-driven task | Task released by a discrete signal rather than a fixed period. |
| Graceful degradation | Reduced functionality under load without destabilizing the control path. |
| Slack denial | Decision not to start lower-priority work because it would risk missing a higher-priority deadline. |
| HAL | Hardware Abstraction Layer — interface decoupling firmware logic from MCU specifics. |
| ISR | Interrupt Service Routine — minimal bounded work in interrupt context. |
| DMA | Direct Memory Access — hardware engine moving data between peripherals and RAM without CPU. |
| Enqueue-or-fail | I/O policy: submit to DMA queue or discard with log — never block. |
| Ring buffer | Circular bounded buffer with wraparound indexing. |
| Double buffer | Two-slot scheme allowing safe producer/consumer state handoff. |
| Heap allocation | Dynamic runtime memory allocation — prohibited on the fast loop critical path. |
| Static allocation | Memory fixed at compile or startup time. |
| Monotonic clock | Clock that never moves backward. |
| Tick | One completed execution of the major cycle. |
| tick_id | Monotonically incrementing counter that increments exactly once per tick. |

---

## 9. Communications and External Interface Terms

| Term | Meaning |
|------|---------|
| Telemetry | Runtime data sent off-board for monitoring, diagnostics, and control. |
| GCS | Ground Control Station — operator-facing software. |
| Companion computer | Onboard higher-level computer communicating with the flight controller over UART or similar. |
| Downlink | Data flowing from vehicle to ground or companion system. |
| Uplink | Data flowing from ground or companion system to vehicle. |
| MAVLink | Lightweight drone messaging protocol widely used for telemetry and commands. |
| DroneCAN | CAN-based protocol for drone peripherals (formerly UAVCAN v0). |
| UART | Serial byte-stream interface. |
| SPI | Synchronous high-speed peripheral bus, used for fast sensors. |
| I2C | Two-wire bus used for slower sensors and peripherals. |
| CAN | Controller Area Network — robust bus for vehicle-level peripheral networking. |
| Framing | Organizing a byte stream into decodable message boundaries with sync, length, and checksum. |
| CRC | Cyclic Redundancy Check for corruption detection. |
| Back-pressure | Pressure from limited link capacity causing traffic reduction or dropping. |
| Bounded write | Transmit submission with a known upper runtime cost; never blocks. |
| Schema versioning | Incrementally versioned binary record layout with forward/backward compatibility rules. |
| Heartbeat | Periodic presence announcement containing system state summary. |
| Stream rate | Configured frequency at which a particular telemetry message is sent. |
| Parameter protocol | Exchange of configuration values between vehicle and GCS. |

---

## 10. DroneOS Source Mappings

Quick reference mapping domain concepts to source directories:

| Concept | Source location |
|---------|----------------|
| Deterministic scheduler | `src/runtime/` |
| Attitude estimator | `src/estimation/` |
| Rate loop PID | `src/control/ControlLoop.*` |
| Wrench allocation | `src/control/ControlAllocator.*` |
| AI command ingress | `src/comms/AiControlLink.*` |
| Binary telemetry | `src/comms/TelemetryPublisher.*` |
| Safety state machine | `src/safety/` |
| Sensor abstractions | `src/sensing/` |
| Parameter registry | `src/config/` |
| SD log storage | `src/logging/` |
| HAL interfaces | `src/platform/` |
| Shared types | `src/common/` |

---

Quick links: [Docs Index](README.md) | [Architecture](architecture.md) | [Technical Concepts](technical-concepts.md) | [Top](#droneos--domain-glossary)
