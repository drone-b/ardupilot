# DroneFirmware

DroneFirmware is a proprietary embedded firmware starter for multi-domain robotic vehicles.
It is designed as an original architecture that borrows only high-level patterns common to
modern autopilot systems: layered abstractions, deterministic scheduling, isolated control
loops, interface-driven hardware access, and explicit safety ownership.

## Design goals

- Hardware-agnostic runtime through a narrow HAL contract
- Predictable timing for sensing, estimation, control, and actuation
- Extensible component graph for air, ground, and marine products
- Clean separation between platform services and vehicle behavior
- Future-ready integration with Drone Factory configuration and autonomy services

## Starter contents

- `docs/architecture.md`: conceptual analysis and the proposed proprietary architecture
- `src/runtime`: startup and scheduler foundation
- `src/platform`: hardware abstraction interfaces
- `src/sensing`: sensor contracts and manager-facing data types
- `src/control`: control loop placeholders and vehicle command contracts
- `src/common`: shared time and result types

This starter is intentionally small. It establishes module boundaries first so
subsequent work can add estimators, vehicle profiles, communications, and safety policies
without reworking the core runtime.
