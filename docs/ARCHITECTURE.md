# Architecture

## Repository layout

The STM32F401RCT6 and ESP32-C3 directories are independent PlatformIO
projects. Neither project imports source files from the other.

## Primary STM32 path

The intended runtime flow is:

```text
AS5600 -> Position estimator -> Control/effect request
       -> Safety limiter -> Motor driver -> BTS7960 -> RS-775

PC USB input/output <-> USB device layer <-> Control/effect processing
```

The current STM32 baseline implements the sensor, estimator, serial test
request, safety limiter and motor-driver portions. USB CDC is available for
diagnostics. Native HID steering reports and HID PID output reports remain
future stages.

Responsibilities in the STM32 project are split between:

- hardware abstraction and fail-safe motor output;
- AS5600 acquisition and diagnostics;
- multi-turn position and velocity estimation;
- safety state and watchdog handling;
- centering, soft-lock and serial-force test control;
- persistent versioned configuration;
- telemetry and command handling;
- hardware-independent logic tests.

## ESP32-C3 prototype

The ESP32-C3 baseline combines most logic in a single Arduino source file and
uses USB serial/JTAG for diagnostics and test commands. Because ESP32-C3 does
not provide the required general-purpose native USB device path, the prototype
is not an HID PID target. It remains useful for historical comparison and
serial-control experiments.

## Future USB boundary

USB HID input, USB CDC diagnostics and HID PID effect output must remain
separate from the motor-safety layer. Loss of USB traffic or malformed reports
must result in a bounded request and safe motor stop; USB code must never write
PWM registers directly.

