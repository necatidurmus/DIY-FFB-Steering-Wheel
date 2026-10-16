#ifndef WHEEL_HAL_H
#define WHEEL_HAL_H

#include "wheel_config.h"

#if defined(F401_RCT6_USB_CDC)
#include <USBSerial.h>
#define WHEEL_SERIAL SerialUSB
#else
#define WHEEL_SERIAL Serial
#endif

struct MotorFeedback {
  bool currentAvailable = false;
  float currentAmps = 0.0F;
};

void halInit();
void halStopAllMotion();

// API sozlesmesi: pwmPercent daima 0..100 yuzdedir. Duty donusumu ve mutlak
// donanim limiti yalnizca HAL icinde uygulanir.
void halSetMotor(MotorDirection direction, uint8_t pwmPercent);

SensorResult halSensorRead(uint16_t& rawAngle, uint8_t& status,
                           uint32_t& readDurationUs);
bool halSensorConnected();

bool halConsoleConnected();
bool halEmergencyStopAsserted();
MotorFeedback halMotorFeedback();

void halSafetyHeartbeat();
bool halSafetyInterlockLatched();
void halLatchSafetyInterlock();
bool halClearSafetyInterlock();

void halWatchdogBegin(uint32_t timeoutUs);
void halWatchdogReload();
void halWatchdogSetTimeout(uint32_t timeoutUs);
bool halWasWatchdogReset();

#endif  // WHEEL_HAL_H
