# Flight Modes Glossary

## Scope

This document summarizes common flight-mode and mission-behavior terminology used across ArduPilot-style
autopilot systems, rewritten for the `DroneFirmware` context.

## Core Mode Concepts

| Term | Meaning |
|---|---|
| Flight mode | A top-level behavior that defines how pilot input, guidance, and automation are interpreted. |
| Manual mode | Mode where the pilot directly commands actuators or low-level vehicle response with minimal stabilization. |
| Stabilized mode | Mode where pilot commands are interpreted through stabilization logic rather than directly passed to actuators. |
| Assisted mode | Mode combining pilot authority with automatic stabilization, protection, or hold behavior. |
| Autonomous mode | Mode where navigation or mission logic becomes the primary command source. |
| Failsafe mode | Mode or state entered after loss of required health or command conditions. |
| Transition | Change from one flight mode to another. |
| Mode manager | Logic that selects and enforces the active mode. |
| Mode-specific controller | Control logic that exists only for a particular mode or class of behavior. |

## Common Multirotor-Oriented Modes

| Term | Meaning |
|---|---|
| Stabilize | Pilot commands attitude, while the controller stabilizes roll and pitch. |
| Acro | Pilot commands angular rates directly; often used for aggressive manual flight. |
| AltHold | Altitude-hold mode that stabilizes vertical motion while leaving more manual horizontal control. |
| Loiter | Position-hold mode that attempts to remain near a fixed location. |
| PosHold | Position-hold mode with stronger manual override feel than fully autonomous position modes. |
| Brake | Mode that aggressively reduces velocity and stops translational motion. |
| Sport | High-authority stabilized mode with more direct pilot feel than loitering modes. |
| Drift | Multirotor mode biased toward coordinated, car-like turning behavior. |
| FlowHold | Hold mode using optical flow instead of full GNSS-based positioning. |
| Turtle mode | Recovery mode for attempting to flip an inverted vehicle upright. |

## Autonomous and Mission Modes

| Term | Meaning |
|---|---|
| Auto | Fully mission-driven behavior executing stored commands and waypoints. |
| Guided | Externally directed mode using offboard or GCS-provided targets. |
| RTL | Return-to-launch or return-to-home mode. |
| SmartRTL | Return logic using a remembered path rather than only a direct return line. |
| Circle | Orbiting behavior around a point or target. |
| Follow | Behavior that tracks a moving target or companion source. |
| Land | Controlled descent and landing behavior. |
| Takeoff | Controlled launch mode that brings the vehicle into a safe flight regime. |
| Mission command | A stored autonomous instruction such as waypoint travel, land, or mode change. |
| Reposition | A command changing the active target location during operation. |

## Fixed-Wing and Hybrid Terms

| Term | Meaning |
|---|---|
| FBWA | Fixed-wing stabilized mode with bank/pitch limits and pilot-directed flight. |
| FBWB | Fixed-wing assisted mode with speed/altitude management layered onto pilot input. |
| Cruise | Mode that combines pilot steering with speed or heading assistance. |
| QStabilize | VTOL stabilized mode for quadplane-style hybrids. |
| QHover | VTOL hover mode for hybrid aircraft. |
| QLoiter | VTOL position-hold mode for hybrid aircraft. |
| QRTL | VTOL return mode for hybrid aircraft. |
| QLand | VTOL landing mode. |
| Transition flight | Operating phase where the aircraft changes between hover and forward flight. |
| VTOL assist | Logic that uses vertical propulsion to help a fixed-wing aircraft under difficult conditions. |
| Tailsitter mode | Control behavior specific to aircraft that rotate between vertical and forward attitudes. |
| Tiltrotor mode | Control behavior specific to aircraft with tilting propulsion units. |

## Ground and Marine Terms

| Term | Meaning |
|---|---|
| Hold | Stationary or stopped mode for rover-like vehicles. |
| Steering mode | Mode emphasizing heading and steering behavior over path tracking. |
| Surface mode | Marine/subsea mode that brings the vehicle upward toward the surface. |
| SurfTrak | Surface-tracking behavior that maintains distance to a measured surface. |
| Dock mode | Guided docking or close-approach behavior. |

## Pilot-Input Interpretation Terms

| Term | Meaning |
|---|---|
| Angle command | Pilot input mapped into a target roll or pitch angle. |
| Rate command | Pilot input mapped into a target angular rate. |
| Throttle command | Pilot input mapped into propulsion demand. |
| Yaw-rate command | Pilot input mapped into rotational yaw rate rather than absolute heading. |
| Heading hold | Behavior that maintains a yaw heading when no yaw-rate command is present. |
| Stick mixing | Combining pilot input with autonomous or stabilization outputs. |
| Input shaping | Smoothing or limiting pilot commands before they become control targets. |
| Command limiting | Restricting pilot or mission targets to safe dynamic bounds. |

## Safety-Related Mode Terms

| Term | Meaning |
|---|---|
| Arming | Transition that permits propulsion output. |
| Disarm | Transition that blocks propulsion output. |
| Pre-arm check | Validation before allowing arming. |
| Arming gate | Condition set that must be satisfied before motors may run. |
| Failsafe latch | Safety behavior that remains active until explicitly cleared. |
| Land detector | Logic that decides whether the vehicle is on the ground or water surface. |
| Crash check | Logic used to detect likely loss of control or impact. |
| Emergency stop | Immediate propulsion shutdown path. |

## Mission and Navigation Terms

| Term | Meaning |
|---|---|
| Waypoint | Stored navigation target. |
| Rally point | Alternate recovery location. |
| Home position | Primary reference used for return and safety behavior. |
| Geofence | Spatial operating boundary. |
| Terrain following | Altitude control relative to ground. |
| Terrain avoidance | Navigation behavior that prevents terrain collision. |
| Dead reckoning | Motion estimate used when external references degrade. |
| Precision landing | Landing behavior using a specific close-range target or sensor. |

## Mapping Guidance for DroneFirmware

For the current `DroneFirmware` architecture:

- `STABILIZE` maps naturally to the outer attitude loop feeding the inner rate loop
- `ACRO` maps naturally to direct rate-target generation
- future autonomous modes should sit above `FlightModeManager`
- mission and guidance behaviors should remain separate from low-level stabilization

## Notes

- Mode names often reuse familiar labels across autopilot systems, but exact behavior can differ by vehicle type.
- The safest architectural habit is to treat a mode as a policy bundle:
  - input interpretation
  - target generation
  - safety constraints
  - navigation authority
