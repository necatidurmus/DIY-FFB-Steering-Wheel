#ifndef WHEEL_CONTROL_H
#define WHEEL_CONTROL_H

#include "config_store.h"
#include "wheel_config.h"
#include "wheel_hal.h"

class WheelController {
 public:
  WheelController();

  void begin(uint32_t nowMs);
  void update(uint32_t nowMs);
  void handleCommand(char* line, uint32_t nowMs);

  void printHelp() const;
  void printStatus(uint32_t nowMs) const;

  bool isSensorHealthy() const { return sensorHealthy_; }
  MotionOwner activeOwner() const { return motorGate_.owner(); }

 private:
  wheel::WheelSettings settings_ = wheel::defaultSettings();
  ConfigStore configStore_;
  wheel::PositionEstimator estimator_;
  wheel::MotorGate motorGate_;
  wheel::SlewLimiter outputSlewLimiter_;

  bool sensorHealthy_ = false;
  bool sensorLinkWasLost_ = false;
  bool calibrationRequired_ = true;
  SensorResult lastSensorError_ = SensorResult::kOk;
  uint8_t lastMagnetStatus_ = 0U;
  uint32_t lastSensorErrorMs_ = 0U;
  uint32_t previousSensorSampleUs_ = 0U;
  uint32_t lastSensorReadDurationUs_ = 0U;
  uint32_t maximumSensorReadDurationUs_ = 0U;
  uint32_t sensorSampleCount_ = 0U;
  uint32_t sensorErrorCount_ = 0U;
  uint32_t sensorReconnectCount_ = 0U;

  uint32_t nextControlUs_ = 0U;
  uint32_t nextSensorUs_ = 0U;
  uint32_t lastControlUs_ = 0U;
  uint32_t lastTelemetryMs_ = 0U;
  uint32_t missedControlCycles_ = 0U;
  uint32_t maximumControlLatenessUs_ = 0U;
  uint32_t lastLoopMs_ = 0U;

  bool positionControlActive_ = false;
  float targetAngleDegrees_ = 0.0F;
  float moveStartAngleDegrees_ = 0.0F;
  float moveInitialErrorDegrees_ = 0.0F;
  float lastProgressAngleDegrees_ = 0.0F;
  uint32_t moveStartedMs_ = 0U;
  uint32_t lastProgressMs_ = 0U;

  bool centeringActive_ = false;
  uint32_t centeringStartedMs_ = 0U;
  float centeringKpTerm_ = 0.0F;
  float centeringKdTerm_ = 0.0F;
  float requestedOutputPercent_ = 0.0F;
  float appliedOutputPercent_ = 0.0F;

  bool ffbControlActive_ = false;
  int16_t ffbCommandPercent_ = 0;
  uint32_t lastFfbCommandMs_ = 0U;
  uint32_t ffbStartedMs_ = 0U;

  bool telemetryEnabled_ = true;
  bool usbWasConnected_ = false;
  bool safetyFaultReported_ = false;
  bool watchdogResetAtBoot_ = false;
  bool persistentConfigLoaded_ = false;

  char commandBuffer_[48] = {};
  uint8_t commandLength_ = 0U;

  void processSerial(uint32_t nowMs);
  void updateSafety(uint32_t nowMs);
  void updateScheduledTasks(uint32_t nowMs, uint32_t nowUs);
  void updateSensor(uint32_t nowMs, uint32_t nowUs);
  void updatePositionController(uint32_t nowMs, uint32_t elapsedUs);
  void updateCenteringController(uint32_t nowMs, uint32_t elapsedUs);
  void updateFfbController(uint32_t nowMs, uint32_t elapsedUs);
  void serviceMotor(uint32_t nowMs);

  bool automaticMotionAllowed(const char* operation);
  void stopMotorOutputs();
  void stopAllMotion();
  void requestManualMotor(MotorDirection direction, uint8_t pwmPercent,
                          uint32_t nowMs);
  void commandContinuousMotor(float signedOutputPercent, MotionOwner owner,
                              uint32_t nowMs);
  void applyMotorCommand(const wheel::MotorCommand& command);

  void startPositionMove(float requestedAngle, uint32_t nowMs);
  void abortPositionMove(const char* reason);
  void startCentering(uint32_t nowMs);
  void abortCentering(const char* reason);
  void setFfbCommand(int16_t commandPercent, uint32_t nowMs);
  void zeroCurrentPosition();
  void loadSafeRuntimeDefaults();

  void reportSensorError(SensorResult result, uint32_t nowMs);
  const char* sensorErrorMessage(SensorResult result) const;
  const char* ownerName() const;
  void printConfiguration() const;
  bool saveConfiguration();
  bool loadConfiguration();

  static char* skipSpaces(char* text);
  static bool hasOnlyTrailingSpaces(char* text);
  static bool matchesWord(const char* text, const char* word);
  static bool parseFloatArgument(char* argument, float& value);
  static bool parseLongArgument(char* argument, long& value);
};

#endif  // WHEEL_CONTROL_H
