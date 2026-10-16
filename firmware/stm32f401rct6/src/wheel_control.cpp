#include "wheel_control.h"

#include <Arduino.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

WheelController::WheelController() = default;

void WheelController::begin(uint32_t nowMs) {
  halInit();
  watchdogResetAtBoot_ = halWasWatchdogReset();
  persistentConfigLoaded_ = configStore_.load(settings_);
  if (!persistentConfigLoaded_) {
    settings_ = wheel::defaultSettings();
  }

  WHEEL_SERIAL.println();
  WHEEL_SERIAL.println("STM32F401RCT6 guvenli direksiyon firmware'i");
#if defined(F401_RCT6_USB_CDC)
  WHEEL_SERIAL.println("Konsol: native USB FS CDC (SerialUSB)");
#else
  WHEEL_SERIAL.println("Konsol: UART Serial (PA2 TX / PA3 RX)");
#endif
  WHEEL_SERIAL.println(
      "ACILIS GUVENLI: motor kapali; 'z' ve yon dogrulamasi olmadan otomatik hareket yok");
  WHEEL_SERIAL.println(persistentConfigLoaded_
                           ? "Kalici konfigurasyon CRC/gecerlilik kontrolunden gecti"
                           : "Kalici konfigurasyon yok/bozuk; guvenli varsayilanlar kullaniliyor");
  if (watchdogResetAtBoot_) {
    WHEEL_SERIAL.println("UYARI: Onceki reset IWDG watchdog kaynakliydi");
  }

  const uint32_t nowUs = micros();
  updateSensor(nowMs, nowUs);
  nextControlUs_ = nowUs + Timing::kControlPeriodUs;
  nextSensorUs_ = nowUs + Timing::kSensorPeriodUs;
  lastControlUs_ = nowUs;
  lastTelemetryMs_ = nowMs;
  lastLoopMs_ = nowMs;
  usbWasConnected_ = halConsoleConnected();
  printConfiguration();
  printHelp();

  halSafetyHeartbeat();
  halWatchdogBegin(Timing::kIwdgTimeoutUs);
}

void WheelController::update(uint32_t nowMs) {
  processSerial(nowMs);  // 's' komutu her seyden once islenir.
  updateSafety(nowMs);

  const uint32_t nowUs = micros();
  updateScheduledTasks(nowMs, nowUs);
  serviceMotor(nowMs);

  if (wheel::timeReached(nowMs, lastTelemetryMs_,
                         Timing::kTelemetryPeriodMs)) {
    lastTelemetryMs_ = nowMs;
    printStatus(nowMs);
  }

  lastLoopMs_ = nowMs;
  halSafetyHeartbeat();
  halWatchdogReload();
}

void WheelController::processSerial(uint32_t nowMs) {
  uint8_t processed = 0U;
  while (WHEEL_SERIAL.available() > 0 &&
         processed < Console::kMaximumBytesPerLoop) {
    ++processed;
    const char received = static_cast<char>(WHEEL_SERIAL.read());

    // Tek basina gelen s/S gecikmeden guvenli durusa gecer. Bu fiziksel bir
    // E-stop degildir; USB ve CPU calisiyor olmalidir.
    if (commandLength_ == 0U && (received == 's' || received == 'S')) {
      const bool wasBusy = motorGate_.active() || motorGate_.pending() ||
                           positionControlActive_ || centeringActive_ ||
                           ffbControlActive_;
      stopAllMotion();
      WHEEL_SERIAL.println(wasBusy ? "Motor durduruldu"
                                   : "Motor zaten duruyor");
      continue;
    }

    if (received == '\r' || received == '\n') {
      if (commandLength_ > 0U) {
        commandBuffer_[commandLength_] = '\0';
        handleCommand(commandBuffer_, nowMs);
        commandLength_ = 0U;
      }
    } else if (commandLength_ < sizeof(commandBuffer_) - 1U) {
      commandBuffer_[commandLength_++] = received;
    } else {
      commandLength_ = 0U;
      stopAllMotion();
      WHEEL_SERIAL.println("HATA: Komut cok uzun; motor guvenli durusa alindi");
    }
  }
}

void WheelController::updateSafety(uint32_t nowMs) {
  (void)nowMs;
  const bool consoleConnected = halConsoleConnected();
  if (usbWasConnected_ && !consoleConnected) {
    stopAllMotion();
  } else if (!usbWasConnected_ && consoleConnected) {
    WHEEL_SERIAL.println("USB/UART konsol baglantisi kuruldu; motor kapali");
    printHelp();
  }
  usbWasConnected_ = consoleConnected;

  if (halEmergencyStopAsserted()) {
    halLatchSafetyInterlock();
  }

  if (halSafetyInterlockLatched()) {
    stopAllMotion();
    if (!safetyFaultReported_) {
      WHEEL_SERIAL.println(
          "HATA: Guvenlik kilidi aktif (E-stop veya kontrol dongusu gecikmesi)");
      WHEEL_SERIAL.println("Nedeni giderin; kilidi acmak icin 'x' gonderin");
      safetyFaultReported_ = true;
    }
  }
}

void WheelController::updateScheduledTasks(uint32_t nowMs, uint32_t nowUs) {
  if (static_cast<int32_t>(nowUs - nextControlUs_) < 0) {
    return;
  }

  const uint32_t latenessUs = nowUs - nextControlUs_;
  if (latenessUs > maximumControlLatenessUs_) {
    maximumControlLatenessUs_ = latenessUs;
  }
  if (latenessUs >= Timing::kControlPeriodUs) {
    missedControlCycles_ += latenessUs / Timing::kControlPeriodUs;
    nextControlUs_ = nowUs + Timing::kControlPeriodUs;
  } else {
    nextControlUs_ += Timing::kControlPeriodUs;
  }

  const uint32_t elapsedControlUs = nowUs - lastControlUs_;
  lastControlUs_ = nowUs;

  if (static_cast<int32_t>(nowUs - nextSensorUs_) >= 0) {
    const uint32_t sensorLatenessUs = nowUs - nextSensorUs_;
    nextSensorUs_ = sensorLatenessUs >= Timing::kSensorPeriodUs
                        ? nowUs + Timing::kSensorPeriodUs
                        : nextSensorUs_ + Timing::kSensorPeriodUs;
    updateSensor(nowMs, nowUs);
  }

  if (positionControlActive_) {
    updatePositionController(nowMs, elapsedControlUs);
  } else if (centeringActive_) {
    updateCenteringController(nowMs, elapsedControlUs);
  } else if (ffbControlActive_) {
    updateFfbController(nowMs, elapsedControlUs);
  }
}

void WheelController::updateSensor(uint32_t nowMs, uint32_t nowUs) {
  uint16_t rawAngle = 0U;
  uint8_t status = 0U;
  uint32_t durationUs = 0U;
  const SensorResult result = halSensorRead(rawAngle, status, durationUs);
  lastSensorReadDurationUs_ = durationUs;
  if (durationUs > maximumSensorReadDurationUs_) {
    maximumSensorReadDurationUs_ = durationUs;
  }

  if (result != SensorResult::kOk) {
    ++sensorErrorCount_;
    reportSensorError(result, nowMs);
    previousSensorSampleUs_ = 0U;
    return;
  }

  ++sensorSampleCount_;
  lastMagnetStatus_ = status;
  const bool recovering = !sensorHealthy_ && sensorLinkWasLost_;

  if (!estimator_.initialized()) {
    int32_t initialCounts = 0;
    if (settings_.sensorCalibrated != 0U) {
      initialCounts = wheel::unwrapDelta(rawAngle, settings_.centerRaw) *
                      settings_.encoderDirectionSign;
    }
    // AS5600 tek turludur. Kalici centerRaw olsa bile acilista 360 derece
    // disindaki tur sayisi bilinemez; bu nedenle trusted=false.
    estimator_.initialize(rawAngle, initialCounts, false);
    calibrationRequired_ = true;
  } else if (recovering) {
    estimator_.resynchronize(rawAngle);
    calibrationRequired_ = true;
    ++sensorReconnectCount_;
    WHEEL_SERIAL.println(
        "AS5600 yeniden baglandi; kopukken hareket bilinemez, 'z' yeniden gerekli");
  } else {
    const uint32_t elapsedUs = previousSensorSampleUs_ == 0U
                                   ? Timing::kSensorPeriodUs
                                   : nowUs - previousSensorSampleUs_;
    estimator_.update(rawAngle, elapsedUs, settings_.encoderDirectionSign,
                      Timing::kVelocityFilterTimeConstantSeconds);
  }

  previousSensorSampleUs_ = nowUs;
  sensorHealthy_ = true;
  sensorLinkWasLost_ = false;
  lastSensorError_ = SensorResult::kOk;
}

void WheelController::reportSensorError(SensorResult result, uint32_t nowMs) {
  const bool motionWasActive = motorGate_.active() || motorGate_.pending();
  stopAllMotion();
  estimator_.invalidate();
  sensorHealthy_ = false;
  sensorLinkWasLost_ = true;
  calibrationRequired_ = true;

  if (motionWasActive) {
    WHEEL_SERIAL.println("Motor AS5600 guvenlik hatasi nedeniyle durduruldu");
  }
  if (result != lastSensorError_ ||
      wheel::timeReached(nowMs, lastSensorErrorMs_,
                         Timing::kErrorRepeatPeriodMs)) {
    WHEEL_SERIAL.println(sensorErrorMessage(result));
    lastSensorErrorMs_ = nowMs;
  }
  lastSensorError_ = result;
}

const char* WheelController::sensorErrorMessage(SensorResult result) const {
  switch (result) {
    case SensorResult::kCommunicationError:
      return "HATA: AS5600 I2C haberlesmesi basarisiz";
    case SensorResult::kMagnetNotDetected:
      return "HATA: AS5600 miknayt algilamiyor";
    case SensorResult::kMagnetTooWeak:
      return "HATA: AS5600 miknayt alani cok zayif";
    case SensorResult::kMagnetTooStrong:
      return "HATA: AS5600 miknayt alani cok guclu";
    case SensorResult::kOk:
      return "OK";
  }
  return "HATA: Bilinmeyen sensor durumu";
}

bool WheelController::automaticMotionAllowed(const char* operation) {
  if (!sensorHealthy_ || !estimator_.initialized()) {
    stopAllMotion();
    WHEEL_SERIAL.print("HATA: ");
    WHEEL_SERIAL.print(operation);
    WHEEL_SERIAL.println(" icin saglikli AS5600 gerekli");
    return false;
  }
  if (calibrationRequired_ || !estimator_.trusted() ||
      settings_.sensorCalibrated == 0U) {
    stopAllMotion();
    WHEEL_SERIAL.print("HATA: ");
    WHEEL_SERIAL.print(operation);
    WHEEL_SERIAL.println(" icin direksiyon merkezdeyken 'z' gerekli");
    return false;
  }
  if (settings_.directionValidated == 0U) {
    stopAllMotion();
    WHEEL_SERIAL.print("HATA: ");
    WHEEL_SERIAL.print(operation);
    WHEEL_SERIAL.println(" icin dusuk guc yon testi ve 'v' onayi gerekli");
    return false;
  }
  if (halSafetyInterlockLatched() || halEmergencyStopAsserted()) {
    stopAllMotion();
    WHEEL_SERIAL.println("HATA: Guvenlik kilidi aktif");
    return false;
  }
  return true;
}

void WheelController::stopMotorOutputs() {
  motorGate_.stop();
  halStopAllMotion();
  appliedOutputPercent_ = 0.0F;
}

void WheelController::stopAllMotion() {
  positionControlActive_ = false;
  centeringActive_ = false;
  ffbControlActive_ = false;
  ffbCommandPercent_ = 0;
  centeringKpTerm_ = 0.0F;
  centeringKdTerm_ = 0.0F;
  requestedOutputPercent_ = 0.0F;
  appliedOutputPercent_ = 0.0F;
  outputSlewLimiter_.reset();
  stopMotorOutputs();
}

void WheelController::applyMotorCommand(const wheel::MotorCommand& command) {
  halSetMotor(command.direction, command.pwmPercent);
  appliedOutputPercent_ =
      command.direction == MotorDirection::kLeft
          ? -static_cast<float>(command.pwmPercent)
          : static_cast<float>(command.pwmPercent);
}

void WheelController::requestManualMotor(MotorDirection direction,
                                         uint8_t pwmPercent,
                                         uint32_t nowMs) {
  stopAllMotion();
  if (!sensorHealthy_ || !estimator_.initialized()) {
    WHEEL_SERIAL.println(
        "HATA: Manuel test icin dahi saglikli AS5600 gerekli");
    return;
  }
  if (halSafetyInterlockLatched() || halEmergencyStopAsserted()) {
    WHEEL_SERIAL.println("HATA: Guvenlik kilidi aktif");
    return;
  }

  if (pwmPercent > settings_.maximumPwmPercent) {
    pwmPercent = settings_.maximumPwmPercent;
  }
  wheel::MotorCommand output{};
  (void)motorGate_.request(direction, pwmPercent, MotionOwner::kManual, nowMs,
                           Motor::kManualDeadTimeMs, output);
  halStopAllMotion();
  WHEEL_SERIAL.print("Manuel motor beklemede: ");
  WHEEL_SERIAL.print(direction == MotorDirection::kRight ? "SAG" : "SOL");
  WHEEL_SERIAL.print(" | uygulanan limit %");
  WHEEL_SERIAL.println(pwmPercent);
}

void WheelController::commandContinuousMotor(float signedOutputPercent,
                                             MotionOwner owner,
                                             uint32_t nowMs) {
  signedOutputPercent = wheel::clampFloat(
      signedOutputPercent, -static_cast<float>(settings_.maximumPwmPercent),
      static_cast<float>(settings_.maximumPwmPercent));
  requestedOutputPercent_ = signedOutputPercent;

  if (fabsf(signedOutputPercent) <
      CenteringControl::kOutputOffThresholdPercent) {
    stopMotorOutputs();
    return;
  }

  const MotorDirection direction = wheel::directionForDemand(
      signedOutputPercent, settings_.motorDirectionSign);
  const uint8_t pwmPercent = static_cast<uint8_t>(
      wheel::clampFloat(fabsf(signedOutputPercent) + 0.5F, 1.0F,
                        settings_.maximumPwmPercent));
  wheel::MotorCommand output{};
  const wheel::MotorGateAction action = motorGate_.request(
      direction, pwmPercent, owner, nowMs,
      Motor::kControlDirectionDeadTimeMs, output);
  if (action == wheel::MotorGateAction::kApply) {
    applyMotorCommand(output);
  } else {
    halStopAllMotion();
    appliedOutputPercent_ = 0.0F;
  }
}

void WheelController::serviceMotor(uint32_t nowMs) {
  if (motorGate_.pending()) {
    if (!sensorHealthy_ || halSafetyInterlockLatched() ||
        halEmergencyStopAsserted()) {
      stopAllMotion();
      return;
    }
    wheel::MotorCommand output{};
    if (motorGate_.service(nowMs, output) ==
        wheel::MotorGateAction::kApply) {
      applyMotorCommand(output);
      if (output.owner == MotionOwner::kManual) {
        WHEEL_SERIAL.println("Manuel motor aktif; 500 ms timeout basladi");
      }
    }
  }

  if (motorGate_.owner() == MotionOwner::kManual && motorGate_.active() &&
      wheel::timeReached(nowMs, motorGate_.activeSince(),
                         Motor::kMaximumManualRunTimeMs)) {
    stopAllMotion();
    WHEEL_SERIAL.println("Motor otomatik durduruldu: 500 ms zaman asimi");
  }
}

void WheelController::startPositionMove(float requestedAngle,
                                        uint32_t nowMs) {
  if (!automaticMotionAllowed("hedef konum")) {
    return;
  }
  if (!isfinite(requestedAngle) ||
      requestedAngle < PositionControl::kMinimumTargetDegrees ||
      requestedAngle > PositionControl::kMaximumTargetDegrees) {
    WHEEL_SERIAL.println("HATA: Hedef -90 ile +90 derece arasinda olmali");
    return;
  }

  const float currentAngle = estimator_.angleDegrees();
  if (fabsf(currentAngle) > PositionControl::kMaximumSafePositionDegrees) {
    stopAllMotion();
    WHEEL_SERIAL.println("HATA: Mevcut konum hedef kontrol siniri disinda");
    return;
  }

  stopAllMotion();
  targetAngleDegrees_ = requestedAngle;
  moveStartAngleDegrees_ = currentAngle;
  moveInitialErrorDegrees_ = fabsf(targetAngleDegrees_ - currentAngle);
  lastProgressAngleDegrees_ = currentAngle;
  moveStartedMs_ = nowMs;
  lastProgressMs_ = nowMs;
  if (moveInitialErrorDegrees_ <= PositionControl::kToleranceDegrees) {
    WHEEL_SERIAL.println("Direksiyon zaten hedef toleransi icinde");
    return;
  }
  positionControlActive_ = true;
  WHEEL_SERIAL.print("Hedef konum basladi: ");
  WHEEL_SERIAL.println(targetAngleDegrees_, 2);
}

void WheelController::abortPositionMove(const char* reason) {
  stopAllMotion();
  WHEEL_SERIAL.print("HATA: Hedef hareket iptal: ");
  WHEEL_SERIAL.println(reason);
}

void WheelController::updatePositionController(uint32_t nowMs,
                                               uint32_t elapsedUs) {
  if (!sensorHealthy_ || !estimator_.trusted()) {
    abortPositionMove("sensor/kalibrasyon gecersiz");
    return;
  }
  const float currentAngle = estimator_.angleDegrees();
  const float error = targetAngleDegrees_ - currentAngle;
  const float absoluteError = fabsf(error);
  if (fabsf(currentAngle) > PositionControl::kMaximumSafePositionDegrees) {
    abortPositionMove("100 derece yazilimsal sinir");
    return;
  }
  if (absoluteError <= PositionControl::kToleranceDegrees) {
    stopAllMotion();
    WHEEL_SERIAL.println("Hedefe ulasildi");
    return;
  }
  if (wheel::timeReached(nowMs, moveStartedMs_,
                         PositionControl::kMoveTimeoutMs)) {
    abortPositionMove("3 saniye zaman asimi");
    return;
  }
  if (fabsf(currentAngle - lastProgressAngleDegrees_) >=
      PositionControl::kProgressThresholdDegrees) {
    lastProgressAngleDegrees_ = currentAngle;
    lastProgressMs_ = nowMs;
  } else if (wheel::timeReached(nowMs, lastProgressMs_,
                                PositionControl::kStallTimeoutMs)) {
    abortPositionMove("motor hareket etmiyor");
    return;
  }
  const float travelled = fabsf(currentAngle - moveStartAngleDegrees_);
  if (travelled >= PositionControl::kDirectionCheckTravelDegrees &&
      absoluteError > moveInitialErrorDegrees_ +
                          PositionControl::kDirectionErrorMarginDegrees) {
    settings_.directionValidated = 0U;
    abortPositionMove("yon hatasi; 'd' ayari ve dusuk guc testi gerekli");
    return;
  }

  const float magnitude = wheel::clampFloat(
      PositionControl::kProportionalGain * absoluteError, 0.0F,
      static_cast<float>(settings_.maximumCenteringPwmPercent));
  const float requested = error > 0.0F ? magnitude : -magnitude;
  const float applied = outputSlewLimiter_.update(
      requested, settings_.slewRatePercentPerSecond, elapsedUs);
  commandContinuousMotor(applied, MotionOwner::kPositionControl, nowMs);
}

void WheelController::startCentering(uint32_t nowMs) {
  if (!automaticMotionAllowed("merkezleme")) {
    return;
  }
  stopAllMotion();
  lastControlUs_ = micros();
  centeringActive_ = true;
  centeringStartedMs_ = nowMs;
  WHEEL_SERIAL.print("Aktif merkezleme acildi | mod ");
  WHEEL_SERIAL.print(settings_.steeringRotationDegrees);
  WHEEL_SERIAL.print(" | guc limiti %");
  WHEEL_SERIAL.println(settings_.maximumCenteringPwmPercent);
  (void)nowMs;
}

void WheelController::abortCentering(const char* reason) {
  stopAllMotion();
  WHEEL_SERIAL.print("HATA: Merkezleme kapatildi: ");
  WHEEL_SERIAL.println(reason);
}

void WheelController::updateCenteringController(uint32_t nowMs,
                                                uint32_t elapsedUs) {
  if (!sensorHealthy_ || !estimator_.trusted()) {
    abortCentering("sensor/kalibrasyon gecersiz");
    return;
  }
  if (wheel::timeReached(nowMs, centeringStartedMs_,
                         Timing::kMaximumAutomaticSessionMs)) {
    abortCentering("30 saniye guvenli test oturumu doldu");
    return;
  }

  const float angle = estimator_.angleDegrees();
  const float velocity = estimator_.velocityDegreesPerSecond();
  const float absoluteAngle = fabsf(angle);
  float springScale = 1.0F;
  if (absoluteAngle <= CenteringControl::kCenterDeadbandDegrees) {
    springScale = 0.0F;
  } else if (absoluteAngle < CenteringControl::kSoftZoneEndDegrees) {
    springScale = wheel::smoothstep(
        (absoluteAngle - CenteringControl::kCenterDeadbandDegrees) /
        (CenteringControl::kSoftZoneEndDegrees -
         CenteringControl::kCenterDeadbandDegrees));
  }
  centeringKpTerm_ = settings_.centeringKp * angle * springScale;
  centeringKdTerm_ = settings_.centeringKd * velocity;

  const wheel::CenteringParameters parameters{
      settings_.centeringKp,
      settings_.centeringKd,
      CenteringControl::kCenterDeadbandDegrees,
      CenteringControl::kSoftZoneEndDegrees,
      static_cast<float>(settings_.maximumCenteringPwmPercent)};
  float requested = wheel::computeCenteringDemand(angle, velocity, parameters);
  requested = wheel::applyVirtualEndstop(
      requested, angle, settings_.steeringRotationDegrees,
      CenteringControl::kVirtualEndstopRampDegrees,
      settings_.maximumEndstopPwmPercent);

  if (fabsf(requested) < CenteringControl::kOutputOffThresholdPercent) {
    outputSlewLimiter_.reset();
    stopMotorOutputs();
    requestedOutputPercent_ = 0.0F;
    return;
  }
  const float applied = outputSlewLimiter_.update(
      requested, settings_.slewRatePercentPerSecond, elapsedUs);
  commandContinuousMotor(applied, MotionOwner::kCentering, nowMs);
}

void WheelController::setFfbCommand(int16_t commandPercent,
                                    uint32_t nowMs) {
  if (!automaticMotionAllowed("FFB seri test")) {
    return;
  }
  if (!ffbControlActive_) {
    stopAllMotion();
    ffbControlActive_ = true;
    ffbStartedMs_ = nowMs;
    WHEEL_SERIAL.println(
        "FFB seri test modu aktif (USB HID PID degildir, gercek tork degildir)");
  }
  ffbCommandPercent_ = commandPercent;
  lastFfbCommandMs_ = nowMs;
  if (commandPercent == 0) {
    outputSlewLimiter_.reset();
    stopMotorOutputs();
  }
}

void WheelController::updateFfbController(uint32_t nowMs,
                                          uint32_t elapsedUs) {
  if (!sensorHealthy_ || !estimator_.trusted()) {
    stopAllMotion();
    WHEEL_SERIAL.println("HATA: Sensor nedeniyle FFB kapatildi");
    return;
  }
  if (wheel::timeReached(nowMs, lastFfbCommandMs_,
                         Timing::kFfbWatchdogTimeoutMs)) {
    stopAllMotion();
    WHEEL_SERIAL.println("HATA: FFB watchdog 100 ms; motor kapatildi");
    return;
  }
  if (wheel::timeReached(nowMs, ffbStartedMs_,
                         Timing::kMaximumAutomaticSessionMs)) {
    stopAllMotion();
    WHEEL_SERIAL.println(
        "HATA: FFB 30 saniye guvenli test oturumu doldu; motor kapatildi");
    return;
  }

  float requested = wheel::clampFloat(
      static_cast<float>(ffbCommandPercent_),
      -static_cast<float>(settings_.maximumPwmPercent),
      static_cast<float>(settings_.maximumPwmPercent));
  requested = wheel::applyVirtualEndstop(
      requested, estimator_.angleDegrees(), settings_.steeringRotationDegrees,
      CenteringControl::kVirtualEndstopRampDegrees,
      settings_.maximumEndstopPwmPercent);
  if (fabsf(requested) < CenteringControl::kOutputOffThresholdPercent) {
    outputSlewLimiter_.reset();
    stopMotorOutputs();
    return;
  }
  const float applied = outputSlewLimiter_.update(
      requested, settings_.slewRatePercentPerSecond, elapsedUs);
  commandContinuousMotor(applied, MotionOwner::kFfb, nowMs);
}

void WheelController::zeroCurrentPosition() {
  if (motorGate_.active() || motorGate_.pending() || positionControlActive_ ||
      centeringActive_ || ffbControlActive_) {
    WHEEL_SERIAL.println("HATA: Sifirlamadan once 's' ile motoru durdurun");
    return;
  }
  if (!sensorHealthy_ || !estimator_.initialized()) {
    WHEEL_SERIAL.println("HATA: Sifirlama icin saglikli AS5600 gerekli");
    return;
  }
  estimator_.zero(estimator_.raw());
  settings_.centerRaw = estimator_.raw();
  settings_.sensorCalibrated = 1U;
  calibrationRequired_ = false;
  WHEEL_SERIAL.println(
      "Merkez sifirlandi (RAM); kalici yapmak icin 'cfg save' gonderin");
}

void WheelController::loadSafeRuntimeDefaults() {
  stopAllMotion();
  const int8_t motorSign = settings_.motorDirectionSign;
  const int8_t encoderSign = settings_.encoderDirectionSign;
  const uint16_t centerRaw = settings_.centerRaw;
  const uint8_t calibrated = settings_.sensorCalibrated;
  const uint8_t directionValidated = settings_.directionValidated;
  settings_ = wheel::defaultSettings();
  settings_.motorDirectionSign = motorSign;
  settings_.encoderDirectionSign = encoderSign;
  settings_.centerRaw = centerRaw;
  settings_.sensorCalibrated = calibrated;
  settings_.directionValidated = directionValidated;
  WHEEL_SERIAL.println(
      "Guvenli varsayilan kazanc/limitler yuklendi; yon ve kalibrasyon korundu");
  WHEEL_SERIAL.println("'a' artik motoru otomatik baslatmaz");
}

bool WheelController::saveConfiguration() {
  stopAllMotion();
  halWatchdogSetTimeout(Timing::kFlashOperationIwdgTimeoutUs);
  halWatchdogReload();
  const bool saved = configStore_.save(settings_);
  halWatchdogSetTimeout(Timing::kIwdgTimeoutUs);
  halWatchdogReload();
  if (halClearSafetyInterlock()) {
    safetyFaultReported_ = false;
  }
  persistentConfigLoaded_ = saved;
  return saved;
}

bool WheelController::loadConfiguration() {
  wheel::WheelSettings loaded{};
  if (!configStore_.load(loaded)) {
    return false;
  }
  stopAllMotion();
  settings_ = loaded;
  calibrationRequired_ = true;
  if (estimator_.initialized()) {
    estimator_.resynchronize(estimator_.raw());
  }
  persistentConfigLoaded_ = true;
  return true;
}

void WheelController::handleCommand(char* line, uint32_t nowMs) {
  char* cursor = skipSpaces(line);
  if (*cursor == '\0') {
    return;
  }

  if (matchesWord(cursor, "cfg")) {
    cursor = skipSpaces(cursor + 3);
    if (matchesWord(cursor, "save") &&
        hasOnlyTrailingSpaces(cursor + 4)) {
      WHEEL_SERIAL.println(saveConfiguration()
                               ? "Konfigurasyon Flash'a kaydedildi ve dogrulandi"
                               : "HATA: Konfigurasyon kaydi/dogrulamasi basarisiz");
      return;
    }
    if (matchesWord(cursor, "load") &&
        hasOnlyTrailingSpaces(cursor + 4)) {
      WHEEL_SERIAL.println(loadConfiguration()
                               ? "Konfigurasyon yuklendi; guvenlik geregi 'z' yeniden gerekli"
                               : "HATA: Gecerli kalici konfigurasyon yok");
      return;
    }
    if (matchesWord(cursor, "reset") &&
        hasOnlyTrailingSpaces(cursor + 5)) {
      stopAllMotion();
      settings_ = wheel::defaultSettings();
      calibrationRequired_ = true;
      estimator_.invalidate();
      WHEEL_SERIAL.println(
          "Guvenli fabrika ayarlari RAM'e yuklendi; Flash degismedi");
      return;
    }
    if (matchesWord(cursor, "show") &&
        hasOnlyTrailingSpaces(cursor + 4)) {
      printConfiguration();
      return;
    }
    WHEEL_SERIAL.println("HATA: cfg save/load/reset/show");
    return;
  }

  struct FloatCommand {
    const char* name;
    float minimum;
    float maximum;
    float* destination;
  };
  FloatCommand floatCommands[] = {
      {"kp", 0.0F, 2.0F, &settings_.centeringKp},
      {"kd", 0.0F, 0.20F, &settings_.centeringKd},
      {"rt", 5.0F, 300.0F, &settings_.slewRatePercentPerSecond},
  };
  for (const FloatCommand& item : floatCommands) {
    if (matchesWord(cursor, item.name)) {
      float value = 0.0F;
      if (!parseFloatArgument(cursor + strlen(item.name), value) ||
          value < item.minimum || value > item.maximum) {
        WHEEL_SERIAL.println("HATA: Parametre aralik disi veya gecersiz");
        return;
      }
      stopAllMotion();
      *item.destination = value;
      WHEEL_SERIAL.print(item.name);
      WHEEL_SERIAL.print(" ayarlandi: ");
      WHEEL_SERIAL.println(value, 3);
      return;
    }
  }

  if (matchesWord(cursor, "mx") || matchesWord(cursor, "cp") ||
      matchesWord(cursor, "ef")) {
    const char first = static_cast<char>(tolower(cursor[0]));
    const char second = static_cast<char>(tolower(cursor[1]));
    long value = 0;
    if (!parseLongArgument(cursor + 2, value) || value < 1L || value > 50L) {
      WHEEL_SERIAL.println("HATA: PWM limiti 1..50 olmali");
      return;
    }
    stopAllMotion();
    if (first == 'm' && second == 'x') {
      settings_.maximumPwmPercent = static_cast<uint8_t>(value);
      if (settings_.maximumCenteringPwmPercent > value) {
        settings_.maximumCenteringPwmPercent = static_cast<uint8_t>(value);
      }
      if (settings_.maximumEndstopPwmPercent > value) {
        settings_.maximumEndstopPwmPercent = static_cast<uint8_t>(value);
      }
    } else if (first == 'c') {
      settings_.maximumCenteringPwmPercent = static_cast<uint8_t>(
          value > settings_.maximumPwmPercent ? settings_.maximumPwmPercent
                                              : value);
    } else {
      settings_.maximumEndstopPwmPercent = static_cast<uint8_t>(
          value > settings_.maximumPwmPercent ? settings_.maximumPwmPercent
                                              : value);
    }
    printConfiguration();
    return;
  }

  const char command = static_cast<char>(
      tolower(static_cast<unsigned char>(*cursor++)));
  if (*cursor != '\0' && !isspace(static_cast<unsigned char>(*cursor))) {
    WHEEL_SERIAL.println("Gecersiz komut; 'h' yardim");
    return;
  }
  cursor = skipSpaces(cursor);

  if (command == 's' && hasOnlyTrailingSpaces(cursor)) {
    stopAllMotion();
    WHEEL_SERIAL.println("Motor durduruldu");
  } else if (command == 'h' && hasOnlyTrailingSpaces(cursor)) {
    printHelp();
  } else if (command == 'a' && hasOnlyTrailingSpaces(cursor)) {
    loadSafeRuntimeDefaults();
  } else if (command == 'z' && hasOnlyTrailingSpaces(cursor)) {
    zeroCurrentPosition();
  } else if (command == 'x' && hasOnlyTrailingSpaces(cursor)) {
    stopAllMotion();
    if (halClearSafetyInterlock()) {
      safetyFaultReported_ = false;
      WHEEL_SERIAL.println("Guvenlik kilidi temizlendi; motor kapali");
    } else {
      WHEEL_SERIAL.println("HATA: Fiziksel E-stop hala aktif");
    }
  } else if (command == 'v' && hasOnlyTrailingSpaces(cursor)) {
    if (!sensorHealthy_ || !estimator_.trusted()) {
      WHEEL_SERIAL.println("HATA: Once merkezde 'z' gerekli");
    } else if (motorGate_.active() || motorGate_.pending()) {
      WHEEL_SERIAL.println("HATA: Yon onayindan once motoru durdurun");
    } else {
      settings_.directionValidated = 1U;
      WHEEL_SERIAL.println(
          "Motor/enkoder yonu kullanici tarafindan onaylandi (cfg save ile kalici)");
    }
  } else if (command == 'd' && hasOnlyTrailingSpaces(cursor)) {
    stopAllMotion();
    settings_.motorDirectionSign *= -1;
    settings_.directionValidated = 0U;
    WHEEL_SERIAL.println(
        "Motor yon isareti terslendi; dusuk guc testinden sonra 'v' gerekli");
  } else if (command == 'e' && hasOnlyTrailingSpaces(cursor)) {
    stopAllMotion();
    settings_.encoderDirectionSign *= -1;
    settings_.sensorCalibrated = 0U;
    settings_.directionValidated = 0U;
    calibrationRequired_ = true;
    if (estimator_.initialized()) {
      estimator_.resynchronize(estimator_.raw());
    }
    WHEEL_SERIAL.println("Enkoder yonu terslendi; 'z' ve yon testi yeniden gerekli");
  } else if (command == 'c') {
    if (*cursor == '\0' ||
        (matchesWord(cursor, "on") && hasOnlyTrailingSpaces(cursor + 2))) {
      startCentering(nowMs);
    } else if (matchesWord(cursor, "off") &&
               hasOnlyTrailingSpaces(cursor + 3)) {
      stopAllMotion();
      WHEEL_SERIAL.println("Aktif merkezleme kapatildi");
    } else {
      WHEEL_SERIAL.println("HATA: c on/off");
    }
  } else if (command == 't') {
    if (matchesWord(cursor, "on") && hasOnlyTrailingSpaces(cursor + 2)) {
      telemetryEnabled_ = true;
      WHEEL_SERIAL.println("Telemetri acildi");
    } else if (matchesWord(cursor, "off") &&
               hasOnlyTrailingSpaces(cursor + 3)) {
      telemetryEnabled_ = false;
      WHEEL_SERIAL.println("Telemetri kapatildi");
    } else {
      WHEEL_SERIAL.println("HATA: t on/off");
    }
  } else if (command == 'm') {
    if (*cursor == '\0') {
      WHEEL_SERIAL.print("Direksiyon modu: ");
      WHEEL_SERIAL.println(settings_.steeringRotationDegrees);
    } else {
      long value = 0;
      if (!parseLongArgument(cursor, value) ||
          (value != 360L && value != 900L && value != 1080L)) {
        WHEEL_SERIAL.println("HATA: m 360/900/1080");
      } else {
        stopAllMotion();
        settings_.steeringRotationDegrees = static_cast<uint16_t>(value);
        WHEEL_SERIAL.println("Direksiyon modu ayarlandi; motor kapali");
      }
    }
  } else if (command == 'g') {
    float value = 0.0F;
    if (!parseFloatArgument(cursor, value)) {
      WHEEL_SERIAL.println("HATA: Ornek g 10");
    } else {
      startPositionMove(value, nowMs);
    }
  } else if (command == 'f') {
    long value = 0;
    if (!parseLongArgument(cursor, value) || value < -100L || value > 100L) {
      stopAllMotion();
      WHEEL_SERIAL.println("HATA: f -100..100; motor kapatildi");
    } else {
      setFfbCommand(static_cast<int16_t>(value), nowMs);
    }
  } else if (command == 'r' || command == 'l') {
    long value = Motor::kDefaultManualPwmPercent;
    if (*cursor != '\0' && !parseLongArgument(cursor, value)) {
      WHEEL_SERIAL.println("HATA: PWM tam sayi olmali");
      return;
    }
    if (value < 1L || value > 50L) {
      WHEEL_SERIAL.println("HATA: Manuel PWM 1..50 olmali");
      return;
    }
    requestManualMotor(command == 'r' ? MotorDirection::kRight
                                       : MotorDirection::kLeft,
                       static_cast<uint8_t>(value), nowMs);
  } else {
    WHEEL_SERIAL.println("Gecersiz komut; 'h' yardim");
  }
}

void WheelController::printHelp() const {
  WHEEL_SERIAL.println("Komutlar:");
  WHEEL_SERIAL.println("  s            yazilimsal stop (fiziksel E-stop degil)");
  WHEEL_SERIAL.println("  r/l [1-50]   500 ms manuel yon testi (varsayilan %10)");
  WHEEL_SERIAL.println("  z            mevcut fiziksel merkezi sifirla");
  WHEEL_SERIAL.println("  d / e        motor / enkoder yon isaretini tersle");
  WHEEL_SERIAL.println("  v            dusuk guc yon testini kullanici olarak onayla");
  WHEEL_SERIAL.println("  c on/off     PD merkezlemeyi ac/kapat");
  WHEEL_SERIAL.println("  f -100..100  timeout'lu seri kuvvet testi; HID PID degil");
  WHEEL_SERIAL.println("  g -90..90    sinirli hedef konum testi");
  WHEEL_SERIAL.println("  m 360/900/1080 | kp | kd | rt | mx | cp | ef");
  WHEEL_SERIAL.println("  a            guvenli varsayilan kazanc/limitler; motoru acmaz");
  WHEEL_SERIAL.println("  cfg save/load/reset/show | t on/off | x | h");
}

void WheelController::printConfiguration() const {
  WHEEL_SERIAL.print("CFG range=");
  WHEEL_SERIAL.print(settings_.steeringRotationDegrees);
  WHEEL_SERIAL.print(" kp=");
  WHEEL_SERIAL.print(settings_.centeringKp, 3);
  WHEEL_SERIAL.print(" kd=");
  WHEEL_SERIAL.print(settings_.centeringKd, 3);
  WHEEL_SERIAL.print(" slew=");
  WHEEL_SERIAL.print(settings_.slewRatePercentPerSecond, 1);
  WHEEL_SERIAL.print(" max/center/endstop=");
  WHEEL_SERIAL.print(settings_.maximumPwmPercent);
  WHEEL_SERIAL.print('/');
  WHEEL_SERIAL.print(settings_.maximumCenteringPwmPercent);
  WHEEL_SERIAL.print('/');
  WHEEL_SERIAL.print(settings_.maximumEndstopPwmPercent);
  WHEEL_SERIAL.print(" mdir/edir=");
  WHEEL_SERIAL.print(settings_.motorDirectionSign);
  WHEEL_SERIAL.print('/');
  WHEEL_SERIAL.print(settings_.encoderDirectionSign);
  WHEEL_SERIAL.print(" centerRaw=");
  WHEEL_SERIAL.print(settings_.centerRaw);
  WHEEL_SERIAL.print(" cal/dirOk=");
  WHEEL_SERIAL.print(settings_.sensorCalibrated);
  WHEEL_SERIAL.print('/');
  WHEEL_SERIAL.println(settings_.directionValidated);
}

const char* WheelController::ownerName() const {
  switch (motorGate_.owner()) {
    case MotionOwner::kManual:
      return "MANUAL";
    case MotionOwner::kPositionControl:
      return "POSITION";
    case MotionOwner::kCentering:
      return "CENTER";
    case MotionOwner::kFfb:
      return "SERIAL_FFB";
    case MotionOwner::kNone:
      return motorGate_.pending() ? "DEADTIME" : "OFF";
  }
  return "OFF";
}

void WheelController::printStatus(uint32_t nowMs) const {
  (void)nowMs;
  if (!telemetryEnabled_ || !halConsoleConnected()) {
    return;
  }
  WHEEL_SERIAL.print("TEL angle=");
  WHEEL_SERIAL.print(estimator_.angleDegrees(), 2);
  WHEEL_SERIAL.print(" vel=");
  WHEEL_SERIAL.print(estimator_.velocityDegreesPerSecond(), 1);
  WHEEL_SERIAL.print(" raw=");
  WHEEL_SERIAL.print(estimator_.raw());
  WHEEL_SERIAL.print(" hid=");
  WHEEL_SERIAL.print(wheel::angleToHidAxis(
      estimator_.angleDegrees(), settings_.steeringRotationDegrees,
      estimator_.trusted() && !calibrationRequired_));
  WHEEL_SERIAL.print(" mode=");
  WHEEL_SERIAL.print(ownerName());
  WHEEL_SERIAL.print(" pwm=");
  WHEEL_SERIAL.print(motorGate_.activeCommand().pwmPercent);
  WHEEL_SERIAL.print(" req=");
  WHEEL_SERIAL.print(requestedOutputPercent_, 1);
  WHEEL_SERIAL.print(" cal=");
  WHEEL_SERIAL.print(calibrationRequired_ ? "REQUIRED" : "OK");
  WHEEL_SERIAL.print(" sensor_us=");
  WHEEL_SERIAL.print(lastSensorReadDurationUs_);
  WHEEL_SERIAL.print('/');
  WHEEL_SERIAL.print(maximumSensorReadDurationUs_);
  WHEEL_SERIAL.print(" samples/errors/reconnect=");
  WHEEL_SERIAL.print(sensorSampleCount_);
  WHEEL_SERIAL.print('/');
  WHEEL_SERIAL.print(sensorErrorCount_);
  WHEEL_SERIAL.print('/');
  WHEEL_SERIAL.print(sensorReconnectCount_);
  WHEEL_SERIAL.print(" missed/late_us=");
  WHEEL_SERIAL.print(missedControlCycles_);
  WHEEL_SERIAL.print('/');
  WHEEL_SERIAL.println(maximumControlLatenessUs_);
}

char* WheelController::skipSpaces(char* text) {
  while (*text != '\0' && isspace(static_cast<unsigned char>(*text))) {
    ++text;
  }
  return text;
}

bool WheelController::hasOnlyTrailingSpaces(char* text) {
  return *skipSpaces(text) == '\0';
}

bool WheelController::matchesWord(const char* text, const char* word) {
  while (*word != '\0') {
    if (tolower(static_cast<unsigned char>(*text)) !=
        tolower(static_cast<unsigned char>(*word))) {
      return false;
    }
    ++text;
    ++word;
  }
  return *text == '\0' || isspace(static_cast<unsigned char>(*text));
}

bool WheelController::parseFloatArgument(char* argument, float& value) {
  argument = skipSpaces(argument);
  if (*argument == '\0') {
    return false;
  }
  char* end = nullptr;
  value = strtof(argument, &end);
  return end != argument && hasOnlyTrailingSpaces(end) && isfinite(value);
}

bool WheelController::parseLongArgument(char* argument, long& value) {
  argument = skipSpaces(argument);
  if (*argument == '\0') {
    return false;
  }
  char* end = nullptr;
  value = strtol(argument, &end, 10);
  return end != argument && hasOnlyTrailingSpaces(end);
}
