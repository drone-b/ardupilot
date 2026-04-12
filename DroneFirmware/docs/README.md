# DroneOS — Documentation Index

Navigation: [Docs Index](README.md) | [Architecture](architecture.md) | [Technical Concepts](technical-concepts.md) | [Glossary](glossary.md) | [Specification Index](../../../DroneFirmware/specification/README.md) | [ADR Index](../../../DroneFirmware/specification/adr/README.md) | [Archive](archive/)

This folder contains **developer-facing documentation**: practical guides, domain references, and
technical explanations for engineers working on the firmware source.

For formal specifications (requirements, contracts, ADRs), see [`specification/`](../../../DroneFirmware/specification/).

---

## Documents in this folder

| File | Purpose |
|------|---------|
| [architecture.md](architecture.md) | System overview, subsystem map, execution model — quick orientation for new contributors |
| [technical-concepts.md](technical-concepts.md) | Deep explanations of every technical concept used in the project (wrench, allocator, ESKF, etc.) |
| [glossary.md](glossary.md) | Unified domain glossary — vehicle, control, estimation, realtime, communications terms |
| [DroneOS_master_plan.md](DroneOS_master_plan.md) | Operational development roadmap with milestones, dependencies, risks, and acceptance criteria |
| [stm32h743vit_usb_bringup.md](stm32h743vit_usb_bringup.md) | Safe USB-only bring-up plan for an STM32H743VIT-class board |

---

## When to use docs/ vs specification/

| Question | Go to |
|----------|-------|
| "What does this term mean?" | [glossary.md](glossary.md) |
| "How does this subsystem work conceptually?" | [technical-concepts.md](technical-concepts.md) |
| "Where does this code live and how do I change it?" | [architecture.md](architecture.md) |
| "What are the formal architectural decisions?" | [specification/adr/README.md](../../../DroneFirmware/specification/adr/README.md) |
| "What are the binary interface contracts?" | [specification/architecture-contracts-v1.md](../../../DroneFirmware/specification/architecture-contracts-v1.md) |
| "What is the formal system architecture?" | [specification/architecture-v1.md](../../../DroneFirmware/specification/architecture-v1.md) |
| "What are the verifiable requirements?" | [specification/verifiable-requirements-v1.md](../../../DroneFirmware/specification/verifiable-requirements-v1.md) |
| "Why are we building this?" | [specification/strategy-v1.md](../../../DroneFirmware/specification/strategy-v1.md) |

---

## Repository layout at a glance

```
DroneFirmware/
├── src/
│   ├── app/           Composition root, main loop
│   ├── comms/         Telemetry, AI control link
│   ├── common/        Shared types (timestamps, durations)
│   ├── config/        Parameter registry
│   ├── control/       PID rate loop, ControlAllocator
│   ├── estimation/    AttitudeEstimator (quaternion + bias)
│   ├── logging/       Binary SD log storage
│   ├── platform/      HAL interfaces and SimHal
│   ├── runtime/       Scheduler, cooperative executive
│   ├── safety/        SafetyManager, arming, failsafe
│   └── sensing/       IMU abstraction, SensorManager
├── docs/              ← you are here
└── specification/     Formal specs, ADRs, contracts
```

---

## Specification folder index

| Document | Description |
|----------|-------------|
| `strategy-v1.md` | Vision, differentiators, and long-horizon goals |
| `architecture-v1.md` | Formal layered architecture |
| `architecture-contracts-v1.md` | Binary interface contracts for all entities |
| `glossary-v1.md` | Canonical specification-level glossary |
| `verifiable-requirements-v1.md` | Testable system requirements |
| `safety-fault-matrix-v1.md` | Fault conditions and safety responses |
| `traceability-matrix-v1.md` | Requirements ↔ implementation mapping |
| `adr/` | Architecture Decision Records (0001–0008) |

---

Quick links: [Architecture](architecture.md) | [Technical Concepts](technical-concepts.md) | [Glossary](glossary.md) | [Specification Index](../../../DroneFirmware/specification/README.md) | [ADR Index](../../../DroneFirmware/specification/adr/README.md)
