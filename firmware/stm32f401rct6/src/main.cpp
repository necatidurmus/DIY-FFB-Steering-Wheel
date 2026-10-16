/*============================================================================
  STM32F401RCT6 Direksiyon Kontrolcusu — Ana Girdi Dosyası
  ----------------------------------------------------------------------------
  Bu proje C3 (ESP32-C3) projesinden değerler ve kontrol mantığını alarak,
  STM32F401RCT6 için sıfırdan yeniden yazılmıştır.

  Önemli tasarım kararları:
    - Seri konsol: Native USB FS (PA11=D-, PA12=D+) uzerinden SerialUSB/CDC.
      Board variantinin USB OTG_FS'yi desteklemesi gerekir.
    - Sensör : AS5600 (I2C1: PB7 SDA, PB6 SCL)
    - Motor  : BTS7960 (TIM4_CH3=PB8 RPWM, TIM4_CH4=PB9 LPWM, PB5 Enable)
    - PWM    : 20 kHz, 8-bit
============================================================================*/

#include <Arduino.h>
#include <Wire.h>

#include "wheel_config.h"
#include "wheel_control.h"

WheelController wheelController;

void setup() {
  wheelController.begin(millis());
}

void loop() {
  wheelController.update(millis());
}
