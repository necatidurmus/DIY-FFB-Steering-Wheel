# DIY Force Feedback Steering Wheel

DIY sim racing steering-wheel firmware for an RS-775 brushed DC motor,
BTS7960 H-bridge and AS5600 magnetic encoder.

This repository contains two independent PlatformIO projects:

- `firmware/stm32f401rct6`: active and primary development platform.
- `firmware/esp32c3`: preserved earlier serial-control prototype.

## Current status

### STM32F401RCT6

The current baseline builds with PlatformIO and contains safe motor-output
initialization, AS5600 multi-turn position estimation, time-limited serial
motor tests, centering and virtual-endstop control, watchdogs, persistent
configuration and host-side logic tests.

The firmware currently exposes a USB CDC diagnostic interface. It does **not**
yet enumerate as a USB steering wheel and does not implement USB HID PID force
feedback. Firmware build success is not a substitute for physical motor,
encoder or USB validation.

See [`firmware/stm32f401rct6/README.md`](firmware/stm32f401rct6/README.md) for
the detailed safety and bring-up procedure.

### ESP32-C3 prototype

**ESP32-C3: Archived prototype — Development deferred until STM32F401RCT6 firmware is complete.**

The ESP32-C3 project is retained as the earlier serial-command prototype. Its
built-in USB Serial/JTAG peripheral is not a general-purpose USB device
controller and cannot be used as a native USB HID PID FFB wheel. It is not the
target for the main HID PID implementation. Its current sources are preserved
without fixes, dependency changes or new features. It may be evaluated again
after the STM32F401RCT6 roadmap is complete.

## Planned capabilities

- Native USB HID steering-wheel input on STM32F401RCT6
- USB HID PID force-feedback reports and effect processing
- Game-driven force feedback
- 1080-degree Truck, 900-degree Road/Racing and 360-degree Formula modes
- Soft lock and calibration
- USB configuration and test application
- Motor-current feedback
- Pedals, shifter and steering-wheel buttons

Planned capabilities are not claimed as implemented. Progress is tracked in
[`docs/ROADMAP.md`](docs/ROADMAP.md).

## Hardware overview

- STM32F401RCT6 primary controller
- ESP32-C3 legacy prototype
- RS-775 12 V brushed DC motor
- BTS7960 H-bridge
- AS5600 magnetic encoder
- Belt/geared steering transmission
- Separate 12 V motor supply

See [`docs/HARDWARE.md`](docs/HARDWARE.md) before connecting motor power.

## Building

The active STM32F401RCT6 firmware is built as an independent PlatformIO
project:

```bash
cd firmware/stm32f401rct6
pio run
```

The archived ESP32-C3 directory retains its original independent PlatformIO
project definition for source preservation. It is not part of the active
development or release gate.

## Development workflow

- `main`: verified baseline code
- `develop`: integration work
- `feature/*`: new features and roadmap stages
- `fix/*`: corrective changes

Each major stage should be developed on its own branch, tested, and merged by
pull request. For example:

```bash
git switch -c feature/stage-2-usb-hid
```

Unless the repository owner explicitly requests otherwise, issues, branches,
commits and pull requests must target `firmware/stm32f401rct6/` and must not
modify the ESP32-C3 archive. See [`CONTRIBUTING.md`](CONTRIBUTING.md).

## Safety

This system can apply force directly to a person's hands. Begin every hardware
test with the 12 V motor supply disconnected. Verify PWM, enable, encoder and
direction signals before attaching the motor or transmission. The serial stop
command is not a replacement for a physical emergency-stop circuit, fuse,
current protection and mechanically safe end stops.

