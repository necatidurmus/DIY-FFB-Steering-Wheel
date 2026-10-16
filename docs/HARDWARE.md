# Hardware

## Current system

- Controller: STM32F401RCT6, 84 MHz, 256 KiB Flash, 64 KiB SRAM
- Legacy prototype: ESP32-C3
- Motor: RS-775 brushed DC motor, 12 V
- Driver: BTS7960 H-bridge
- Encoder: AS5600, 12-bit magnetic absolute angle
- Transmission: belt/geared steering mechanism
- Motor supply: separate 12 V supply

## STM32F401RCT6 connections

| Function | Pin | Peripheral |
|---|---:|---|
| AS5600 SDA | PB7 | I2C1 SDA |
| AS5600 SCL | PB6 | I2C1 SCL |
| BTS7960 RPWM | PB8 | TIM4 CH3 |
| BTS7960 LPWM | PB9 | TIM4 CH4 |
| BTS7960 enable | PB5 | GPIO |
| USB D- | PA11 | USB FS |
| USB D+ | PA12 | USB FS |

The exact board oscillator, USB connector wiring, protection components and
boot-mode circuit must be verified against the physical board before claiming
USB compliance.

## Safety requirements

- Test logic first with the 12 V motor supply disconnected.
- Use a fuse and a physical emergency-stop path that disables the driver.
- Ensure the enable input has a hardware-defined safe state during reset.
- Verify RPWM and LPWM are never active together.
- Confirm motor and encoder direction before centering or soft lock.
- Provide mechanical end stops; software soft lock is not a substitute.
- Measure current, supply droop and motor/driver temperature under load.

Pedal, shifter, button, current-sensor and physical E-stop pins are deliberately
not assigned until their electrical design is finalized.

