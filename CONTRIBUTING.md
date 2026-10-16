# Contributing

## Default development target

`firmware/stm32f401rct6/` is the active firmware and the default scope for new
issues, branches, commits and pull requests.

**ESP32-C3: Archived prototype — Development deferred until STM32F401RCT6 firmware is complete.**

Do not modify `firmware/esp32c3/` for routine fixes, formatting, dependency
updates, CI work, USB development or new features. An ESP32-C3 change requires
an explicit instruction from the repository owner. The archive is retained so
the earlier prototype is not lost; it is not permanently abandoned.

## Branches

- `main`: verified baseline
- `develop`: integration work when needed
- `feature/*`: roadmap stages and new features
- `fix/*`: focused corrections

Use one branch per significant STM32F401RCT6 stage or fix. Merge tested work by
pull request; do not force-push shared branches.

## Pull requests

Every firmware pull request should state:

- the STM32F401RCT6 roadmap stage or defect it addresses;
- build and host-test commands actually run;
- physical tests actually performed;
- hardware tests still pending;
- safety behavior affected by the change.

Do not describe compilation as physical validation and do not mark an
unverified roadmap stage complete.
