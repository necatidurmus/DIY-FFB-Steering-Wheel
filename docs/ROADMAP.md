# Roadmap

No stage is marked complete until its firmware, host tests and required
hardware tests are recorded. The immediate priority is Stage 0 through Stage 3
to obtain a real USB HID steering input without weakening motor safety.

- [ ] **Stage 0: Firmware validation**
  - Reproduce both PlatformIO builds from a clean checkout.
  - Record USB, sensor, PWM and watchdog bench-test evidence.
- [ ] **Stage 1: Motor Control & Safety**
  - Validate direction, dead-time, limits, emergency cutoff and thermal/power
    behavior on hardware.
- [ ] **Stage 2: AS5600 & Calibration**
  - Validate magnet diagnostics, multi-turn tracking, zeroing and persistent
    calibration over power cycles and disconnects.
- [ ] **Stage 3: Native USB HID Steering**
  - Implement and verify standards-compliant steering input reports on Linux
    and Windows.
- [ ] **Stage 4: Configuration & Test Application**
  - Add a host tool for safe calibration, diagnostics and configuration.
- [ ] **Stage 5: Universal USB HID PID FFB Engine**
  - Implement descriptors, report handlers, effect allocation and supported
    effect processing with conformance tests.
- [ ] **Stage 6: Motor FFB Integration**
  - Route effect torque requests through the existing safety and motor-control
    layers; characterize motor output rather than treating PWM as torque.
- [ ] **Stage 7: ETS2 / Assetto Corsa / F1 / Forza Testing**
  - Record per-game enumeration, input and supported-effect results.
- [ ] **Stage 8: Current Feedback, Pedals, Shifter & Buttons**
  - Add only after hardware pin assignments and electrical interfaces are
    finalized.

