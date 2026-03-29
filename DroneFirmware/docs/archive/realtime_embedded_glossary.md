# Realtime and Embedded Systems Glossary

## Scope

This document captures common realtime, scheduling, HAL, and embedded-software terms that appear in
ArduPilot-style firmware and are directly relevant to `DroneFirmware`.

## Core Realtime Terms

| Term | Meaning |
|---|---|
| Realtime system | Software system where timing correctness matters as much as functional correctness. |
| Determinism | Property that execution order, timing, and behavior are bounded and predictable. |
| Jitter | Variation in execution timing, sample timing, or communication latency. |
| Latency | Delay between an event occurring and the software responding to it. |
| Throughput | Sustained rate at which work or data is processed. |
| Deadline | Latest acceptable completion time for a task or operation. |
| Deadline miss | Failure to complete work before its expected deadline. |
| Overrun | Task execution that exceeds its configured time budget. |
| Slack | Remaining time before a higher-priority deadline or release becomes due. |
| Bounded execution | Behavior guaranteed to complete within a known upper time limit. |
| Worst-case execution time | Maximum expected runtime of a task under defined conditions. |

## Scheduler Terms

| Term | Meaning |
|---|---|
| Scheduler | Runtime component that decides when tasks execute. |
| Cooperative scheduler | Scheduler where tasks run until they return; there is no forced preemption. |
| Cyclic executive | Fixed-order loop that runs periodic and event-driven jobs according to timing rules. |
| Fixed priority | Static importance ordering among tasks. |
| Priority class | Grouping of tasks by criticality and expected timing. |
| Release time | Time at which a task becomes eligible to run. |
| Release-based scheduling | Scheduling using absolute release timestamps rather than delay chaining. |
| Periodic task | Task released at a fixed interval. |
| Event-driven task | Task released when a specific event is signaled. |
| Task budget | Time allocation intended for one invocation of a task. |
| Skip policy | Scheduler behavior that drops stale work rather than running it late. |
| Backlog catch-up | Running delayed jobs repeatedly to recover lost schedule position. |
| Static task order | Fixed order inside a priority class to reduce jitter and unpredictability. |
| Instrumentation | Runtime measurement of timing or behavior for diagnostics. |
| High-water mark | Maximum observed value, such as task execution time. |

## Interrupt and HAL Terms

| Term | Meaning |
|---|---|
| ISR | Interrupt Service Routine. |
| Interrupt latency | Time between hardware interrupt assertion and ISR execution. |
| Data-ready interrupt | Hardware signal indicating a sensor sample is available. |
| IRQ | Interrupt request line or interrupt event. |
| Deferred work | Work signaled in ISR context but executed later in scheduled context. |
| HAL | Hardware Abstraction Layer. |
| Board support | Code that binds abstract interfaces to a specific PCB, MCU, and peripheral layout. |
| Driver | Low-level software controlling a peripheral or sensor. |
| Capability-oriented interface | Interface exposing what the system can do rather than mirroring MCU registers directly. |
| Bus factory | HAL service that creates access handles for SPI, I2C, UART, and related resources. |
| Clock abstraction | HAL interface providing monotonic timing services. |
| Board control | HAL interface for watchdog, reset, and low-level board management. |

## Time and Timestamp Terms

| Term | Meaning |
|---|---|
| Timestamp | Time marker attached to an event, sample, or packet. |
| Monotonic clock | Clock that never moves backward. |
| Microsecond counter | Fine-resolution monotonic time source common in flight controllers. |
| Sample time | Time when a physical measurement is considered to have occurred. |
| Acquisition time | Time when software finished reading a sensor sample. |
| Sample age | Time elapsed since a sample was produced. |
| Transport latency | Delay between sample generation and software acquisition. |
| Delta time (`dt`) | Time difference used by estimators and controllers. |
| Time anomaly | Unexpected time jump, such as non-monotonic timestamps. |

## Memory and Allocation Terms

| Term | Meaning |
|---|---|
| Heap allocation | Dynamic memory allocation during runtime. |
| Static allocation | Memory fixed at compile time or startup time. |
| Stack allocation | Function-local storage allocated in call context. |
| Fixed-capacity container | Buffer or table with compile-time or startup-fixed size. |
| Ring buffer | Circular buffer with bounded storage and wraparound indexing. |
| Double buffer | Two-slot buffer scheme allowing safe producer/consumer handoff. |
| Overflow policy | Defined behavior when a buffer fills up. |
| Drop policy | Strategy for discarding old or new data when capacity is exceeded. |

## Peripheral and Bus Terms

| Term | Meaning |
|---|---|
| SPI | Synchronous peripheral bus often used for high-rate sensors. |
| I2C | Two-wire bus used for many slower sensors and peripherals. |
| UART | Serial byte-stream interface used for telemetry, GNSS, and radios. |
| CAN | Bus architecture used for robust peripheral networking. |
| DMA | Direct Memory Access engine used to move data without active CPU copying. |
| FIFO | First-In, First-Out hardware or software queue. |
| Enqueue-only write | API that accepts transmit data for later hardware service and returns immediately. |
| Non-blocking write | API that never waits for physical transmission to complete. |
| Busy status | Immediate indication that a driver cannot accept more work right now. |
| Flow control | Mechanism limiting data transmission to what the receiver can absorb. |

## Execution-Context Terms

| Term | Meaning |
|---|---|
| Main loop context | Cooperative scheduled execution context where most logic runs. |
| ISR context | Interrupt context where only minimal bounded work should occur. |
| Background task | Lower-priority work such as telemetry, diagnostics, or storage flush. |
| Fast loop | High-rate control and estimation execution path. |
| Medium loop | Guidance, mode handling, and outer-loop logic. |
| Slow loop | Telemetry, persistence, and maintenance tasks. |
| Critical path | Sequence of work that directly affects stabilization timing. |
| Service path | Lower-criticality work that may be skipped or decimated under load. |

## Concurrency and Synchronization Terms

| Term | Meaning |
|---|---|
| Preemption | Ability to interrupt a running task to execute another. |
| Non-preemptive execution | Execution model where running work is not forcibly interrupted by the scheduler. |
| Thread | Independently scheduled execution context. |
| RTOS | Real-Time Operating System. |
| Mutex | Lock protecting shared resources from concurrent access. |
| Semaphore | Synchronization primitive for signaling or resource ownership. |
| Lock-free handoff | State transfer approach avoiding blocking synchronization primitives. |
| Race condition | Fault caused by unsafe timing between concurrent accesses. |

## Robustness and Safety Terms

| Term | Meaning |
|---|---|
| Watchdog | Hardware or software monitor that detects stalled execution. |
| Health monitoring | Checking whether subsystems are still operating inside valid bounds. |
| Fail-operational | Design goal where limited function continues after some faults. |
| Fail-safe | Design goal where outputs move to a safe condition after faults. |
| Graceful degradation | Reduced functionality under load or fault without destabilizing the control path. |
| Slack denial | Decision not to start lower-priority work because it would interfere with higher-priority deadlines. |
| Stale work | Work that is no longer useful if executed late. |

## Test and Simulation Terms

| Term | Meaning |
|---|---|
| SITL | Software In The Loop simulation. |
| HIL | Hardware In The Loop testing. |
| Bench test | Local validation without full field operation. |
| Replay | Reprocessing recorded data through firmware logic or estimators. |
| Profiling | Measuring execution costs and timing behavior. |

## Mapping Guidance for DroneFirmware

Key `DroneFirmware` equivalents:

- scheduler and timing control -> `src/runtime/`
- HAL contracts -> `src/platform/`
- timestamped sensing -> `src/sensing/`
- bounded telemetry/logging -> `src/comms/` and `src/logging/`

## Notes

- In a cooperative embedded flight stack, strong timing behavior comes from:
  - bounded drivers
  - explicit priorities
  - static memory
  - release-based scheduling
  - refusing to run stale work
