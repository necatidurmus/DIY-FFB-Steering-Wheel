#ifndef WHEEL_LOGIC_H
#define WHEEL_LOGIC_H

#include <stddef.h>
#include <stdint.h>

namespace wheel {

constexpr uint16_t kEncoderCountsPerTurn = 4096U;
constexpr float kDegreesPerCount = 360.0F / kEncoderCountsPerTurn;

enum class MotorDirection : int8_t {
  kStopped = 0,
  kRight = 1,
  kLeft = -1,
};

enum class MotionOwner : uint8_t {
  kNone,
  kManual,
  kPositionControl,
  kCentering,
  kFfb,
};

struct MotorDrive {
  uint8_t rightDuty;
  uint8_t leftDuty;
  bool enable;
};

uint8_t pwmPercentToDuty(uint8_t percent);
MotorDrive makeMotorDrive(MotorDirection direction, uint8_t percent,
                          uint8_t maximumPercent = 100U);
int16_t unwrapDelta(uint16_t currentRaw, uint16_t previousRaw);
bool timeReached(uint32_t now, uint32_t started, uint32_t duration);
float clampFloat(float value, float minimum, float maximum);
float smoothstep(float value);

class PositionEstimator {
 public:
  void initialize(uint16_t rawAngle, int32_t initialCounts, bool trusted);
  void update(uint16_t rawAngle, uint32_t elapsedMicroseconds,
              int8_t encoderDirectionSign, float filterTimeConstantSeconds);
  void resynchronize(uint16_t rawAngle);
  void zero(uint16_t rawAngle);
  void invalidate();

  bool initialized() const { return initialized_; }
  bool trusted() const { return trusted_; }
  uint16_t raw() const { return currentRaw_; }
  int32_t counts() const { return relativeCounts_; }
  float angleDegrees() const;
  float velocityDegreesPerSecond() const { return filteredVelocity_; }

 private:
  bool initialized_ = false;
  bool trusted_ = false;
  uint16_t currentRaw_ = 0;
  uint16_t previousRaw_ = 0;
  int32_t relativeCounts_ = 0;
  float filteredVelocity_ = 0.0F;
};

struct MotorCommand {
  MotorDirection direction = MotorDirection::kStopped;
  uint8_t pwmPercent = 0;
  MotionOwner owner = MotionOwner::kNone;
};

enum class MotorGateAction : uint8_t {
  kStopped,
  kWaiting,
  kApply,
};

class MotorGate {
 public:
  MotorGateAction request(MotorDirection direction, uint8_t pwmPercent,
                          MotionOwner owner, uint32_t now,
                          uint32_t deadTimeMs, MotorCommand& output);
  MotorGateAction service(uint32_t now, MotorCommand& output);
  void stop();

  bool pending() const { return pending_; }
  bool active() const {
    return active_.direction != MotorDirection::kStopped;
  }
  const MotorCommand& activeCommand() const { return active_; }
  MotionOwner owner() const { return active_.owner; }
  uint32_t activeSince() const { return activeSinceMs_; }

 private:
  MotorCommand active_;
  MotorCommand pendingCommand_;
  bool pending_ = false;
  uint32_t deadTimeStartedMs_ = 0;
  uint32_t deadTimeMs_ = 0;
  uint32_t activeSinceMs_ = 0;
};

class SlewLimiter {
 public:
  void reset(float value = 0.0F);
  float update(float requested, float ratePerSecond,
               uint32_t elapsedMicroseconds);
  float value() const { return value_; }

 private:
  float value_ = 0.0F;
};

struct CenteringParameters {
  float kp;
  float kd;
  float deadbandDegrees;
  float softZoneEndDegrees;
  float maximumOutputPercent;
};

float computeCenteringDemand(float angleDegrees,
                             float angularVelocityDegreesPerSecond,
                             const CenteringParameters& parameters);
float applyVirtualEndstop(float requestedOutputPercent, float angleDegrees,
                          uint16_t rotationDegrees, float rampDegrees,
                          float maximumEndstopPercent);
MotorDirection directionForDemand(float signedDemand,
                                  int8_t motorDirectionSign);
int16_t angleToHidAxis(float angleDegrees, uint16_t rotationDegrees,
                       bool calibrated);

struct WheelInputState {
  int16_t steering = 0;
  uint16_t throttle = 0;
  uint16_t brake = 0;
  uint16_t clutch = 0;
  uint32_t buttons = 0;
  int8_t gear = 0;
  bool pedalsAvailable = false;
  bool buttonsAvailable = false;
};

struct WheelSettings {
  float centeringKp;
  float centeringKd;
  float slewRatePercentPerSecond;
  uint16_t steeringRotationDegrees;
  uint16_t centerRaw;
  uint8_t maximumPwmPercent;
  uint8_t maximumCenteringPwmPercent;
  uint8_t maximumEndstopPwmPercent;
  int8_t motorDirectionSign;
  int8_t encoderDirectionSign;
  uint8_t sensorCalibrated;
  uint8_t directionValidated;
  uint8_t reserved[3];
};

constexpr uint32_t kSettingsMagic = 0x57484646UL;  // "WHFF"
constexpr uint16_t kSettingsVersion = 1U;

struct SettingsRecord {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  WheelSettings settings;
  uint32_t crc32;
};

WheelSettings defaultSettings();
bool settingsAreValid(const WheelSettings& settings);
uint32_t crc32(const uint8_t* data, size_t length);
SettingsRecord makeSettingsRecord(const WheelSettings& settings);
bool settingsRecordIsValid(const SettingsRecord& record);

}  // namespace wheel

#endif  // WHEEL_LOGIC_H
