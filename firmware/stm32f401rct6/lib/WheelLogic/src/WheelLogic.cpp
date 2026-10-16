#include "WheelLogic.h"

#include <cmath>
#include <cstring>

namespace wheel {

uint8_t pwmPercentToDuty(uint8_t percent) {
  if (percent > 100U) {
    percent = 100U;
  }
  return static_cast<uint8_t>(
      (static_cast<uint16_t>(percent) * 255U + 50U) / 100U);
}

MotorDrive makeMotorDrive(MotorDirection direction, uint8_t percent,
                          uint8_t maximumPercent) {
  if (maximumPercent > 100U) {
    maximumPercent = 100U;
  }
  if (percent > maximumPercent) {
    percent = maximumPercent;
  }
  if (direction == MotorDirection::kStopped || percent == 0U) {
    return {0U, 0U, false};
  }

  const uint8_t duty = pwmPercentToDuty(percent);
  if (direction == MotorDirection::kRight) {
    return {duty, 0U, true};
  }
  if (direction == MotorDirection::kLeft) {
    return {0U, duty, true};
  }
  return {0U, 0U, false};
}

int16_t unwrapDelta(uint16_t currentRaw, uint16_t previousRaw) {
  int32_t delta = static_cast<int32_t>(currentRaw & 0x0FFFU) -
                  static_cast<int32_t>(previousRaw & 0x0FFFU);
  if (delta > 2048) {
    delta -= 4096;
  } else if (delta < -2048) {
    delta += 4096;
  }
  return static_cast<int16_t>(delta);
}

bool timeReached(uint32_t now, uint32_t started, uint32_t duration) {
  return static_cast<uint32_t>(now - started) >= duration;
}

float clampFloat(float value, float minimum, float maximum) {
  if (value < minimum) {
    return minimum;
  }
  if (value > maximum) {
    return maximum;
  }
  return value;
}

float smoothstep(float value) {
  value = clampFloat(value, 0.0F, 1.0F);
  return value * value * (3.0F - 2.0F * value);
}

void PositionEstimator::initialize(uint16_t rawAngle, int32_t initialCounts,
                                   bool trusted) {
  currentRaw_ = rawAngle & 0x0FFFU;
  previousRaw_ = currentRaw_;
  relativeCounts_ = initialCounts;
  filteredVelocity_ = 0.0F;
  trusted_ = trusted;
  initialized_ = true;
}

void PositionEstimator::update(uint16_t rawAngle,
                               uint32_t elapsedMicroseconds,
                               int8_t encoderDirectionSign,
                               float filterTimeConstantSeconds) {
  rawAngle &= 0x0FFFU;
  if (!initialized_) {
    initialize(rawAngle, 0, false);
    return;
  }

  const int8_t sign = encoderDirectionSign < 0 ? -1 : 1;
  const int32_t delta =
      static_cast<int32_t>(unwrapDelta(rawAngle, previousRaw_)) * sign;
  relativeCounts_ += delta;

  if (elapsedMicroseconds > 0U) {
    const float elapsedSeconds =
        static_cast<float>(elapsedMicroseconds) * 0.000001F;
    const float instantaneousVelocity =
        static_cast<float>(delta) * kDegreesPerCount / elapsedSeconds;
    const float alpha = elapsedSeconds /
                        (filterTimeConstantSeconds + elapsedSeconds);
    filteredVelocity_ += alpha * (instantaneousVelocity - filteredVelocity_);
  }

  previousRaw_ = rawAngle;
  currentRaw_ = rawAngle;
}

void PositionEstimator::resynchronize(uint16_t rawAngle) {
  currentRaw_ = rawAngle & 0x0FFFU;
  previousRaw_ = currentRaw_;
  filteredVelocity_ = 0.0F;
  trusted_ = false;
  initialized_ = true;
}

void PositionEstimator::zero(uint16_t rawAngle) {
  currentRaw_ = rawAngle & 0x0FFFU;
  previousRaw_ = currentRaw_;
  relativeCounts_ = 0;
  filteredVelocity_ = 0.0F;
  trusted_ = true;
  initialized_ = true;
}

void PositionEstimator::invalidate() {
  trusted_ = false;
  filteredVelocity_ = 0.0F;
}

float PositionEstimator::angleDegrees() const {
  return static_cast<float>(relativeCounts_) * kDegreesPerCount;
}

MotorGateAction MotorGate::request(MotorDirection direction,
                                   uint8_t pwmPercent, MotionOwner owner,
                                   uint32_t now, uint32_t deadTimeMs,
                                   MotorCommand& output) {
  if (direction == MotorDirection::kStopped || pwmPercent == 0U ||
      owner == MotionOwner::kNone) {
    stop();
    output = active_;
    return MotorGateAction::kStopped;
  }

  if (active() && active_.direction == direction && active_.owner == owner) {
    active_.pwmPercent = pwmPercent;
    output = active_;
    return MotorGateAction::kApply;
  }

  if (pending_ && pendingCommand_.direction == direction &&
      pendingCommand_.owner == owner) {
    pendingCommand_.pwmPercent = pwmPercent;
    output = {};
    return MotorGateAction::kWaiting;
  }

  active_ = {};
  pendingCommand_ = {direction, pwmPercent, owner};
  pending_ = true;
  deadTimeStartedMs_ = now;
  deadTimeMs_ = deadTimeMs;
  output = {};
  return MotorGateAction::kWaiting;
}

MotorGateAction MotorGate::service(uint32_t now, MotorCommand& output) {
  if (!pending_) {
    output = active_;
    return active() ? MotorGateAction::kApply : MotorGateAction::kStopped;
  }
  if (!timeReached(now, deadTimeStartedMs_, deadTimeMs_)) {
    output = {};
    return MotorGateAction::kWaiting;
  }

  active_ = pendingCommand_;
  pendingCommand_ = {};
  pending_ = false;
  activeSinceMs_ = now;
  output = active_;
  return MotorGateAction::kApply;
}

void MotorGate::stop() {
  active_ = {};
  pendingCommand_ = {};
  pending_ = false;
  deadTimeStartedMs_ = 0;
  deadTimeMs_ = 0;
  activeSinceMs_ = 0;
}

void SlewLimiter::reset(float value) { value_ = value; }

float SlewLimiter::update(float requested, float ratePerSecond,
                          uint32_t elapsedMicroseconds) {
  if (elapsedMicroseconds == 0U || ratePerSecond <= 0.0F) {
    return value_;
  }
  const float maximumChange =
      ratePerSecond * static_cast<float>(elapsedMicroseconds) * 0.000001F;
  const float change = clampFloat(requested - value_, -maximumChange,
                                  maximumChange);
  value_ += change;
  return value_;
}

float computeCenteringDemand(float angleDegrees,
                             float angularVelocityDegreesPerSecond,
                             const CenteringParameters& parameters) {
  const float absoluteAngle = std::fabs(angleDegrees);
  float springScale = 1.0F;
  if (absoluteAngle <= parameters.deadbandDegrees) {
    springScale = 0.0F;
  } else if (absoluteAngle < parameters.softZoneEndDegrees &&
             parameters.softZoneEndDegrees > parameters.deadbandDegrees) {
    springScale = smoothstep(
        (absoluteAngle - parameters.deadbandDegrees) /
        (parameters.softZoneEndDegrees - parameters.deadbandDegrees));
  }

  const float output =
      -(parameters.kp * angleDegrees * springScale +
        parameters.kd * angularVelocityDegreesPerSecond);
  return clampFloat(output, -parameters.maximumOutputPercent,
                    parameters.maximumOutputPercent);
}

float applyVirtualEndstop(float requestedOutputPercent, float angleDegrees,
                          uint16_t rotationDegrees, float rampDegrees,
                          float maximumEndstopPercent) {
  const float limit = rotationDegrees * 0.5F;
  const float absoluteAngle = std::fabs(angleDegrees);
  if (limit <= 0.0F || rampDegrees <= 0.0F ||
      maximumEndstopPercent <= 0.0F ||
      absoluteAngle <= limit - rampDegrees) {
    return requestedOutputPercent;
  }

  const float inwardSign = angleDegrees > 0.0F ? -1.0F : 1.0F;
  if (absoluteAngle >= limit) {
    return inwardSign * maximumEndstopPercent;
  }

  const float position =
      (absoluteAngle - (limit - rampDegrees)) / rampDegrees;
  const float endstopMagnitude = maximumEndstopPercent * smoothstep(position);
  const bool requestedInward =
      requestedOutputPercent == 0.0F ||
      ((requestedOutputPercent > 0.0F) == (inwardSign > 0.0F));
  if (requestedInward) {
    const float magnitude =
        std::fmax(std::fabs(requestedOutputPercent), endstopMagnitude);
    return inwardSign * magnitude;
  }
  return clampFloat(requestedOutputPercent + inwardSign * endstopMagnitude,
                    -maximumEndstopPercent, maximumEndstopPercent);
}

MotorDirection directionForDemand(float signedDemand,
                                  int8_t motorDirectionSign) {
  if (signedDemand == 0.0F) {
    return MotorDirection::kStopped;
  }
  const bool positivePhysical =
      (signedDemand > 0.0F) == (motorDirectionSign >= 0);
  return positivePhysical ? MotorDirection::kRight : MotorDirection::kLeft;
}

int16_t angleToHidAxis(float angleDegrees, uint16_t rotationDegrees,
                       bool calibrated) {
  if (!calibrated || rotationDegrees == 0U) {
    return 0;
  }
  const float halfRange = rotationDegrees * 0.5F;
  const float normalized = clampFloat(angleDegrees / halfRange, -1.0F, 1.0F);
  const float scaled = normalized * 32767.0F;
  return static_cast<int16_t>(scaled >= 0.0F ? scaled + 0.5F : scaled - 0.5F);
}

WheelSettings defaultSettings() {
  WheelSettings settings{};
  settings.centeringKp = 0.40F;
  settings.centeringKd = 0.025F;
  settings.slewRatePercentPerSecond = 80.0F;
  settings.steeringRotationDegrees = 900U;
  settings.centerRaw = 1708U;
  settings.maximumPwmPercent = 30U;
  settings.maximumCenteringPwmPercent = 25U;
  settings.maximumEndstopPwmPercent = 30U;
  settings.motorDirectionSign = -1;
  settings.encoderDirectionSign = 1;
  settings.sensorCalibrated = 0U;
  settings.directionValidated = 0U;
  return settings;
}

bool settingsAreValid(const WheelSettings& settings) {
  const bool rotationValid = settings.steeringRotationDegrees == 360U ||
                             settings.steeringRotationDegrees == 900U ||
                             settings.steeringRotationDegrees == 1080U;
  return std::isfinite(settings.centeringKp) &&
         std::isfinite(settings.centeringKd) &&
         std::isfinite(settings.slewRatePercentPerSecond) &&
         settings.centeringKp >= 0.0F && settings.centeringKp <= 2.0F &&
         settings.centeringKd >= 0.0F && settings.centeringKd <= 0.20F &&
         settings.slewRatePercentPerSecond >= 5.0F &&
         settings.slewRatePercentPerSecond <= 300.0F && rotationValid &&
         settings.centerRaw < kEncoderCountsPerTurn &&
         settings.maximumPwmPercent >= 1U &&
         settings.maximumPwmPercent <= 50U &&
         settings.maximumCenteringPwmPercent <= settings.maximumPwmPercent &&
         settings.maximumEndstopPwmPercent <= settings.maximumPwmPercent &&
         (settings.motorDirectionSign == -1 ||
          settings.motorDirectionSign == 1) &&
         (settings.encoderDirectionSign == -1 ||
          settings.encoderDirectionSign == 1) &&
         settings.sensorCalibrated <= 1U &&
         settings.directionValidated <= 1U;
}

uint32_t crc32(const uint8_t* data, size_t length) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t index = 0; index < length; ++index) {
    crc ^= data[index];
    for (uint8_t bit = 0; bit < 8U; ++bit) {
      const uint32_t mask = static_cast<uint32_t>(
          -static_cast<int32_t>(crc & 1UL));
      crc = (crc >> 1U) ^ (0xEDB88320UL & mask);
    }
  }
  return ~crc;
}

SettingsRecord makeSettingsRecord(const WheelSettings& settings) {
  SettingsRecord record{};
  record.magic = kSettingsMagic;
  record.version = kSettingsVersion;
  record.size = sizeof(WheelSettings);
  record.settings = settings;
  record.crc32 = crc32(reinterpret_cast<const uint8_t*>(&record),
                      offsetof(SettingsRecord, crc32));
  return record;
}

bool settingsRecordIsValid(const SettingsRecord& record) {
  if (record.magic != kSettingsMagic || record.version != kSettingsVersion ||
      record.size != sizeof(WheelSettings) ||
      !settingsAreValid(record.settings)) {
    return false;
  }
  return record.crc32 ==
         crc32(reinterpret_cast<const uint8_t*>(&record),
               offsetof(SettingsRecord, crc32));
}

static_assert(sizeof(WheelSettings) == 28U,
              "WheelSettings layout changed; increment settings version");

}  // namespace wheel
