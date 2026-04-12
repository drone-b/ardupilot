# STM32H743VIT USB-Only Bring-Up Plan

Navigation: [Docs Index](README.md) | [Master Plan](DroneOS_master_plan.md) | [Architecture](architecture.md)

Status: Draft v1  
Scope: Safe first bring-up of a generic STM32H743VIT board with only USB-C connected

---

## 1. Goal

Bring up DroneOS on an STM32H743VIT-class board without sensors, motors, ESCs, RC input, battery, or external peripherals.

This is a **board-alive milestone**, not a flight milestone.

The first target is:

- boot firmware
- verify monotonic timing
- run the cooperative scheduler
- expose USB CDC or equivalent serial telemetry
- run simulated IMU/control/safety paths
- keep all actuator outputs unavailable or disabled

---

## 2. Current USB Observation

At the start of this bring-up, the connected board was **not visible** to the host:

- `lsusb` did not show an STMicroelectronics DFU device, ST-Link device, or USB CDC serial device
- no `/dev/ttyACM*`
- no `/dev/ttyUSB*`
- no `/dev/serial/by-id`

Observed USB devices were only host peripherals such as hub, camera, Bluetooth/Qualcomm device, and keyboard.

This means the immediate next step is physical/boot-mode enumeration, not flashing firmware yet.

---

## 3. Expected USB Modes

Depending on the exact board wiring and boot mode, an STM32H743 board may appear as:

| Mode | Expected Host Behavior | Notes |
|---|---|---|
| ST-Link debug probe | ST-Link USB device | Only if the board includes onboard ST-Link or external ST-Link is connected |
| STM32 ROM DFU bootloader | USB DFU device, often STMicroelectronics VID | Requires BOOT0/boot pins and USB FS wired correctly |
| Application USB CDC | `/dev/ttyACM*` | Only after firmware with USB CDC is already running |
| Power-only | No useful USB device | Happens with charge-only cable, missing USB data wiring, or no USB firmware/bootloader mode |

---

## 4. Physical Checklist

Before firmware work:

- confirm the USB-C cable supports data, not only charging
- confirm the board is powered from USB
- identify the exact board model, not only the MCU package
- identify whether the board has onboard ST-Link
- identify BOOT0 button/jumper/pad
- identify reset button
- identify whether USB-C is wired to STM32 USB FS pins
- identify if any external oscillator is present and its frequency
- identify at least one user LED if available

---

## 5. Safe Initial Target Profile

DroneOS now has a board-profile placeholder:

```cpp
BoardTargetRegistry::stm32h743vit_usb_profile()
```

This profile intentionally declares:

- STM32H7 family
- USB-only bring-up target name
- no SPI DMA yet
- no UART DMA yet
- no CAN yet
- no sensor IRQ router yet
- `0` PWM channels

Therefore:

```cpp
flight_qualification_ready(stm32h743vit_usb_profile()) == false
```

This is intentional. The board is not considered flight-qualified in this phase.

---

## 6. Bring-Up Stages

## Stage 0: Host Enumeration

Goal:

- make the board visible to the host

Commands:

```sh
lsusb
ls -l /dev/serial/by-id /dev/ttyACM* /dev/ttyUSB*
```

Expected success:

- ST-Link, DFU, or CDC device appears

If nothing appears:

- try another USB cable
- try another USB port
- hold BOOT0 while pressing reset
- verify the board has USB data wiring
- use an external ST-Link if the board lacks onboard debugger

## Stage 1: Toolchain and Flash Access

Goal:

- confirm the host can build and program firmware

Candidate tools:

- `arm-none-eabi-gcc`
- `cmake`
- `ninja`
- `openocd`
- `dfu-util`
- `st-flash`

Expected success:

- at least one flashing path is available:
  - ST-Link/OpenOCD
  - STM32 DFU bootloader

## Stage 2: Minimal Firmware Boot

Goal:

- boot minimal firmware
- blink LED if available
- expose USB CDC serial if supported

No sensors.
No PWM.
No motor output.

## Stage 3: DroneOS Runtime Bring-Up

Goal:

- run scheduler
- publish framed binary telemetry at 50 Hz
- run simulated IMU and control path
- log scheduler/control/safety state through USB serial

Expected success:

- host receives deterministic telemetry packets
- scheduler counters remain stable
- safety state remains `DISARMED` unless explicitly simulated

## Stage 4: Peripheral Expansion

Only after Stage 3:

- SPI IMU
- I2C sensors
- PWM/DShot output
- SD logging
- battery monitor
- RC input

---

## 7. Hard Safety Rules

For this target:

- do not connect ESCs
- do not connect motors
- do not attach propellers
- do not treat USB-only firmware as flight firmware
- do not enable arming by default
- do not publish actuator outputs from simulated sensor data

---

## 8. Immediate Blocker

The board is not currently visible over USB.

The next concrete action is to make the host see one of:

- ST-Link
- STM32 DFU
- USB CDC serial

Until then, DroneOS can be prepared in code, but cannot be flashed or validated on the board.
