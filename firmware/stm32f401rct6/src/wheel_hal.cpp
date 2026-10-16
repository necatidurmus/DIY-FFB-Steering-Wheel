#include "wheel_hal.h"

#include <HardwareTimer.h>
#include <IWatchdog.h>
#include <Wire.h>

#if defined(F401_RCT6_USB_CDC)
#include <usbd_cdc_if.h>
#endif

namespace {

HardwareTimer motorPwmTimer(TIM4);
HardwareTimer safetyTimer(TIM5);

volatile uint32_t safetyStaleTicks = 0U;
volatile bool safetyInterlockLatched = false;
volatile bool safetyMonitoringEnabled = false;

void forceMotorOffFromIsr() {
  // PB5=Enable, TIM4 CH3/CH4=PB8/PB9. Bu ISR yalniz F401 donanimina ozeldir.
  GPIOB->BSRR = GPIO_BSRR_BR5;
  TIM4->CCR3 = 0U;
  TIM4->CCR4 = 0U;
}

void safetyTimerCallback() {
  if (!safetyMonitoringEnabled) {
    return;
  }
  if (safetyStaleTicks < UINT32_MAX) {
    ++safetyStaleTicks;
  }
  if (safetyStaleTicks >= Timing::kLoopSafetyDeadlineMs) {
    forceMotorOffFromIsr();
    safetyInterlockLatched = true;
  }
}

void writePwmDuty(uint8_t rightDuty, uint8_t leftDuty) {
  motorPwmTimer.setCaptureCompare(Motor::kRpwmChannel, rightDuty,
                                  RESOLUTION_8B_COMPARE_FORMAT);
  motorPwmTimer.setCaptureCompare(Motor::kLpwmChannel, leftDuty,
                                  RESOLUTION_8B_COMPARE_FORMAT);
}

}  // namespace

void halInit() {
  // Motor cikislari diger tum cevre birimlerinden once guvenli hale gelir.
  pinMode(Pins::kMotorRpwm, OUTPUT);
  pinMode(Pins::kMotorLpwm, OUTPUT);
  pinMode(Pins::kMotorEnable, OUTPUT);
  digitalWrite(Pins::kMotorEnable, LOW);
  digitalWrite(Pins::kMotorRpwm, LOW);
  digitalWrite(Pins::kMotorLpwm, LOW);

  motorPwmTimer.pause();
  motorPwmTimer.setOverflow(Motor::kPwmFrequencyHz, HERTZ_FORMAT);
  motorPwmTimer.setMode(Motor::kRpwmChannel, TIMER_OUTPUT_COMPARE_PWM1,
                        Pins::kMotorRpwm);
  motorPwmTimer.setMode(Motor::kLpwmChannel, TIMER_OUTPUT_COMPARE_PWM1,
                        Pins::kMotorLpwm);
  writePwmDuty(0U, 0U);
  motorPwmTimer.resume();

  if (Pins::kEmergencyStop >= 0) {
    pinMode(static_cast<uint32_t>(Pins::kEmergencyStop), INPUT_PULLUP);
  }

  WHEEL_SERIAL.begin(Console::kBaud);

  Wire.setSDA(Pins::kAs5600Sda);
  Wire.setSCL(Pins::kAs5600Scl);
  Wire.begin();
  Wire.setClock(As5600::kI2cFrequencyHz);

  safetyStaleTicks = 0U;
  safetyInterlockLatched = false;
  safetyMonitoringEnabled = false;
  safetyTimer.pause();
  safetyTimer.setOverflow(1000U, HERTZ_FORMAT);
  safetyTimer.attachInterrupt(safetyTimerCallback);
  safetyTimer.resume();

  halStopAllMotion();
}

void halStopAllMotion() {
  // Enable once kapatilir; sonra iki PWM kanali sifirlanir.
  digitalWrite(Pins::kMotorEnable, LOW);
  writePwmDuty(0U, 0U);
}

void halSetMotor(MotorDirection direction, uint8_t pwmPercent) {
  const wheel::MotorDrive drive = wheel::makeMotorDrive(
      direction, pwmPercent, Motor::kAbsoluteMaximumPwmPercent);

  // Iki yonun ayni anda etkin olamayacagi tek donanim yazma noktasi.
  // Guvenlik ISR'i ile son Enable yazmasi arasinda yarisa izin verme.
  noInterrupts();
  digitalWrite(Pins::kMotorEnable, LOW);
  writePwmDuty(0U, 0U);
  if (!drive.enable || safetyInterlockLatched ||
      halEmergencyStopAsserted()) {
    interrupts();
    return;
  }

  writePwmDuty(drive.rightDuty, drive.leftDuty);
  digitalWrite(Pins::kMotorEnable, HIGH);
  interrupts();
}

SensorResult halSensorRead(uint16_t& rawAngle, uint8_t& status,
                           uint32_t& readDurationUs) {
  const uint32_t startedUs = micros();
  status = 0U;

  // STATUS (0x0B), RAW ANGLE high (0x0C) ve low (0x0D) bitisik oldugu icin
  // tek I2C okumasi kullanilir. Bu, 500 Hz aday dongunun yukunu azaltir.
  Wire.beginTransmission(As5600::kAddress);
  Wire.write(As5600::kStatusRegister);
  if (Wire.endTransmission(false) != 0U) {
    readDurationUs = micros() - startedUs;
    return SensorResult::kCommunicationError;
  }

  const uint8_t received = Wire.requestFrom(As5600::kAddress, 3U);
  if (received != 3U) {
    while (Wire.available() > 0) {
      (void)Wire.read();
    }
    readDurationUs = micros() - startedUs;
    return SensorResult::kCommunicationError;
  }

  status = static_cast<uint8_t>(Wire.read());
  const uint8_t high = static_cast<uint8_t>(Wire.read());
  const uint8_t low = static_cast<uint8_t>(Wire.read());
  readDurationUs = micros() - startedUs;

  if ((status & As5600::kStatusMagnetDetected) == 0U) {
    return SensorResult::kMagnetNotDetected;
  }
  if ((status & As5600::kStatusMagnetTooWeak) != 0U) {
    return SensorResult::kMagnetTooWeak;
  }
  if ((status & As5600::kStatusMagnetTooStrong) != 0U) {
    return SensorResult::kMagnetTooStrong;
  }

  rawAngle = (static_cast<uint16_t>(high & 0x0FU) << 8U) |
             static_cast<uint16_t>(low);
  return SensorResult::kOk;
}

bool halSensorConnected() {
  Wire.beginTransmission(As5600::kAddress);
  return Wire.endTransmission() == 0U;
}

bool halConsoleConnected() {
#if defined(F401_RCT6_USB_CDC)
  return CDC_connected();
#else
  return true;
#endif
}

bool halEmergencyStopAsserted() {
  return Pins::kEmergencyStop >= 0 &&
         digitalRead(static_cast<uint32_t>(Pins::kEmergencyStop)) == LOW;
}

MotorFeedback halMotorFeedback() {
  // Bu donanimda akim sensoru yoktur. Gercek akim/tork olculuyormus gibi
  // bir deger uretilmez.
  return {};
}

void halSafetyHeartbeat() {
  noInterrupts();
  safetyStaleTicks = 0U;
  safetyMonitoringEnabled = true;
  interrupts();
}

bool halSafetyInterlockLatched() { return safetyInterlockLatched; }

void halLatchSafetyInterlock() {
  noInterrupts();
  safetyInterlockLatched = true;
  interrupts();
  halStopAllMotion();
}

bool halClearSafetyInterlock() {
  if (halEmergencyStopAsserted()) {
    return false;
  }
  noInterrupts();
  safetyStaleTicks = 0U;
  safetyInterlockLatched = false;
  interrupts();
  halStopAllMotion();
  return true;
}

void halWatchdogBegin(uint32_t timeoutUs) { IWatchdog.begin(timeoutUs); }

void halWatchdogReload() { IWatchdog.reload(); }

void halWatchdogSetTimeout(uint32_t timeoutUs) {
  IWatchdog.set(timeoutUs);
}

bool halWasWatchdogReset() { return IWatchdog.isReset(true); }
