# Roadmap

This roadmap applies to the active STM32F401RCT6 platform. No stage is marked
complete until its firmware, host tests and required hardware tests are
recorded. ESP32-C3 development remains deferred until this roadmap is complete
and the repository owner explicitly reactivates it.

- [ ] **Stage 0: Firmware validation**
  - Reproduce the STM32 PlatformIO builds and host tests from a clean checkout.
  - Record USB, sensor, PWM and watchdog bench evidence.
- [ ] **Stage 1: Motor control and AS5600 improvements**
  - Validate direction, dead-time, limits, emergency cutoff, magnet
    diagnostics, multi-turn tracking and safe calibration.
- [ ] **Stage 2: Native USB HID steering**
  - Implement and verify standards-compliant steering input reports on Linux
    and Windows.
- [ ] **Stage 3: USB HID configuration and test interface**
  - Add a host-facing interface and tool for safe calibration, diagnostics and
    configuration.
- [ ] **Stage 4: USB HID PID Force Feedback**
  - Implement descriptors, report handlers, effect allocation and supported
    effect processing, then route requests through the existing motor safety
    layer.
- [ ] **Stage 5: ETS2 / Assetto Corsa / F1 / Forza testing**
  - Record per-game enumeration, input and supported-effect results.
- [ ] **Stage 6: Calibration and performance optimization**
  - Validate persistent calibration, control-loop timing, filtering, thermal
    behavior and safe force tuning.
- [ ] **Stage 7: Required hardware expansions**
  - Add current feedback, pedals, shifter and buttons only after their
    electrical interfaces and pin assignments are finalized.

ESP32-C3 development may be scheduled only after these STM32F401RCT6 stages
are complete and a new explicit instruction is given.

