# MAVLink and GCS Glossary

## Scope

This document captures communication, telemetry, mission, and ground-control terminology commonly used in
ArduPilot-style systems and relevant to future `DroneFirmware` integrations.

## Core Communication Terms

| Term | Meaning |
|---|---|
| Telemetry | Runtime data sent off-board for monitoring, debugging, and command exchange. |
| Downlink | Data flowing from vehicle to ground or companion system. |
| Uplink | Data flowing from ground or companion system to vehicle. |
| GCS | Ground Control Station. |
| Companion computer | Onboard higher-level computer communicating with the flight controller. |
| Link | Physical or logical communication path such as UART radio, USB, or network tunnel. |
| Channel | Logical communication endpoint or stream. |
| Transport | Mechanism that carries serialized messages. |
| Framing | Organizing a byte stream into decodable message boundaries. |
| Packet | Encoded message unit transmitted over a link. |
| Payload | Application data carried inside a framed packet. |
| CRC | Error-detection field used to reject corrupted packets. |

## MAVLink Terms

| Term | Meaning |
|---|---|
| MAVLink | Lightweight messaging protocol widely used in drone systems. |
| MAVLink message | Structured protocol message with ID, payload, and framing overhead. |
| MAVLink channel | Specific logical output/input path for MAVLink transport. |
| MAVLink routing | Forwarding messages between links, endpoints, or components. |
| System ID | Numeric identifier for a vehicle/system on a MAVLink network. |
| Component ID | Identifier for a subsystem within a MAVLink system. |
| Heartbeat | Periodic message announcing presence and mode/state information. |
| Stream rate | Configured frequency at which certain telemetry messages are sent. |
| Message interval | Requested send period for a particular MAVLink message. |
| Payload size check | Validation that enough transmit space exists before packing a message. |
| MAVLink1 / MAVLink2 | Protocol generations with different framing and capability behavior. |
| Command ACK | Message acknowledging result of a command request. |
| In-progress ACK | Response indicating a long-running command is still being processed. |
| Mission protocol | MAVLink message set used to exchange waypoints and mission items. |
| Parameter protocol | MAVLink message set used to exchange configuration parameters. |
| FTP over MAVLink | File transfer mechanism carried inside MAVLink messages. |

## GCS Interaction Terms

| Term | Meaning |
|---|---|
| Ground station | Operator-facing software used to configure, monitor, and command a vehicle. |
| Telemetry radio | Link device used to send telemetry and commands between vehicle and GCS. |
| Console / debug port | Serial interface used for development or diagnostics. |
| Link loss | Communication failure between flight controller and external control system. |
| GCS failsafe | Safety response to loss of required GCS communication. |
| Text status message | Human-readable message sent to operators. |
| Status stream | Regular telemetry carrying health, state, and estimator information. |
| Log download | Transfer of stored logs from the vehicle to offboard tooling. |
| Parameter download | Bulk retrieval of configuration values. |
| Parameter set | Offboard update of a parameter value. |
| Mission upload | Transfer of mission items to the vehicle. |
| Mission download | Retrieval of stored mission items from the vehicle. |

## UART and Serial-Link Terms

| Term | Meaning |
|---|---|
| UART | Serial transport interface often used for telemetry radios and GNSS. |
| Baud rate | Serial signaling speed. |
| TX space | Remaining transmit capacity available in a serial output path. |
| RX buffer | Receive-side temporary storage for incoming bytes. |
| Driver FIFO | Bounded queue inside a UART driver. |
| DMA-backed transmit | UART transmission path offloaded through DMA. |
| Enqueue-only transmit | Non-blocking serial write that only submits bytes to the driver path. |
| Flow control | Mechanism preventing overruns between sender and receiver. |
| Locked port | Serial port reserved for exclusive use by one subsystem. |

## Mission and Command Terms

| Term | Meaning |
|---|---|
| Mission item | One autonomous instruction in a mission sequence. |
| DO command | Mission item that performs an action rather than navigating to a point. |
| NAV command | Mission item primarily related to vehicle movement. |
| Reposition command | Offboard request to change the current navigation target. |
| Mode-change command | External request to change the active vehicle mode. |
| Arm/disarm command | Explicit external request changing propulsion authorization state. |
| Guided target | Externally provided command target used by guided behavior. |

## Telemetry-Rate and Scheduling Terms

| Term | Meaning |
|---|---|
| Telemetry scheduler | Logic deciding which messages are sent and how often. |
| Rate limiter | Mechanism preventing a subsystem from flooding a link. |
| Back-pressure | Pressure from limited link capacity causing traffic reduction or dropping. |
| Opportunistic send | Sending only when there is space and timing budget available. |
| Bounded write | Immediate transmit submission with a known upper runtime cost. |
| Deferred send | Transmission requested now but completed later by a driver or DMA engine. |

## Parameter and Configuration Terms

| Term | Meaning |
|---|---|
| Parameter metadata | Information describing type, range, units, and defaults. |
| Parameter schema | Versioned layout of supported parameters. |
| Default value | Factory or initial value for a parameter. |
| Persistent storage | Non-volatile memory used to retain parameters across reboot. |
| Compatibility version | Version marker used to validate stored parameter layout. |

## Logging and Diagnostics Terms

| Term | Meaning |
|---|---|
| Dataflash log | Persistent binary log format commonly associated with ArduPilot logging. |
| Event log | Log record describing discrete system events. |
| Performance log | Log record describing scheduler load, runtime, or timing behavior. |
| Tuning log | Log fields useful for gain adjustment and control diagnostics. |
| Offline analysis | Post-flight inspection of logs or telemetry captures. |

## Mapping Guidance for DroneFirmware

For `DroneFirmware`, these terms map naturally into:

- transport-neutral telemetry/log publishing -> `src/comms/`
- future external protocol integration -> `integrations/`
- static parameter registry -> `src/config/`
- persistent log backend -> `src/logging/`

## Notes

- MAVLink is an ecosystem-level protocol, not only a packet format.
- Even if `DroneFirmware` eventually uses a different external protocol, this glossary remains useful because:
  - the operational concepts are the same
  - the rate-control and bounded-transmit concerns remain the same
  - GCS interaction patterns remain similar
