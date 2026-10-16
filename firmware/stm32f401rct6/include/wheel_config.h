#ifndef WHEEL_CONFIG_H
#define WHEEL_CONFIG_H

#include <Arduino.h>
#include <WheelLogic.h>
#include <stdint.h>

namespace Pins {
constexpr uint32_t kAs5600Sda = PB7;
constexpr uint32_t kAs5600Scl = PB6;
constexpr uint32_t kMotorRpwm = PB8;
constexpr uint32_t kMotorLpwm = PB9;
constexpr uint32_t kMotorEnable = PB5;

// -1: fiziksel E-stop girisi henuz atanmadi. Bir pin atandiginda aktif LOW
// anahtar ve harici pull-up kullanilir. BTS7960 enable hattinda harici
// pull-down ve tercihen donanimsal kesme zorunlu guvenlik katmanidir.
constexpr int32_t kEmergencyStop = -1;
}  // namespace Pins

namespace Console {
constexpr uint32_t kBaud = 115200U;
constexpr uint8_t kMaximumBytesPerLoop = 32U;
}  // namespace Console

namespace As5600 {
constexpr uint8_t kAddress = 0x36U;
constexpr uint8_t kStatusRegister = 0x0BU;
constexpr uint8_t kStatusMagnetDetected = 1U << 5;
constexpr uint8_t kStatusMagnetTooWeak = 1U << 4;
constexpr uint8_t kStatusMagnetTooStrong = 1U << 3;
constexpr uint32_t kI2cFrequencyHz = 400000U;
}  // namespace As5600

namespace Motor {
constexpr uint8_t kRpwmChannel = 3U;
constexpr uint8_t kLpwmChannel = 4U;
constexpr uint32_t kPwmFrequencyHz = 20000U;
// Donanım HAL katmanı %0-100 arası PWM komutunu desteklemeli.
// Mod special limitleri kontrolcü katmanında uygulanır, HAL'de değil.
constexpr uint8_t kAbsoluteMaximumPwmPercent = 100U;
constexpr uint8_t kDefaultManualPwmPercent = 22U;
constexpr uint8_t kMinimumPwmPercent = 1U;
constexpr uint8_t kMaximumPwmPercent = 50U;
constexpr uint32_t kManualDeadTimeMs = 50U;
constexpr uint32_t kControlDirectionDeadTimeMs = 2U;
constexpr uint32_t kMaximumManualRunTimeMs = 500U;
}  // namespace Motor

namespace Timing {
constexpr uint32_t kControlPeriodUs = 2000U;  // aday 500 Hz; telemetri olcer
constexpr uint32_t kSensorPeriodUs = 2000U;
constexpr uint32_t kTelemetryPeriodMs = 250U;
constexpr uint32_t kErrorRepeatPeriodMs = 1000U;
constexpr uint32_t kLoopSafetyDeadlineMs = 50U;
constexpr uint32_t kFfbWatchdogTimeoutMs = 100U;
constexpr uint32_t kMaximumAutomaticSessionMs = 30000U;
constexpr uint32_t kIwdgTimeoutUs = 250000U;
constexpr uint32_t kFlashOperationIwdgTimeoutUs = 5000000U;
constexpr float kVelocityFilterTimeConstantSeconds = 0.025F;
}  // namespace Timing

namespace PositionControl {
constexpr float kMinimumTargetDegrees = -90.0F;
constexpr float kMaximumTargetDegrees = 90.0F;
constexpr float kMaximumSafePositionDegrees = 100.0F;
constexpr float kToleranceDegrees = 2.0F;
constexpr float kProportionalGain = 0.20F;
constexpr uint8_t kMinimumPwmPercent = 22;
constexpr uint8_t kMaximumPwmPercent = 50;
constexpr uint32_t kMoveTimeoutMs = 3000U;
constexpr uint32_t kStallTimeoutMs = 700U;
constexpr float kProgressThresholdDegrees = 0.50F;
constexpr float kDirectionCheckTravelDegrees = 2.0F;
constexpr float kDirectionErrorMarginDegrees = 1.0F;
}  // namespace PositionControl

namespace CenteringControl {
constexpr float kCenterDeadbandDegrees = 0.5F;
constexpr float kSoftZoneEndDegrees = 3.0F;
constexpr float kMinimumEffectivePwmPercent = 22.0F;
constexpr float kMaximumPdPwmPercent = 30.0F;
constexpr float kDefaultKp = 0.40F;
constexpr float kDefaultKd = 0.025F;
constexpr float kDefaultSlewRatePercentPerSecond = 80.0F;
constexpr float kOutputOffThresholdPercent = 0.5F;

constexpr float kVirtualEndstopRampDegrees = 20.0F;
constexpr float kEndstopStartPwmPercent = 30.0F;
constexpr float kEndstopMaximumPwmPercent = 100.0F;
}  // namespace CenteringControl

enum class SensorResult : uint8_t {
  kOk,
  kCommunicationError,
  kMagnetNotDetected,
  kMagnetTooWeak,
  kMagnetTooStrong,
};

using MotorDirection = wheel::MotorDirection;
using MotionOwner = wheel::MotionOwner;

#endif  // WHEEL_CONFIG_H
