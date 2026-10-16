#include <Arduino.h>
#include <Wire.h>

#if defined(ARDUINO_ARCH_STM32)
#include <HardwareTimer.h>
#endif

#include <ctype.h>
#include <math.h>
#include <stdlib.h>

namespace Pins {
#if defined(ARDUINO_ARCH_STM32)
constexpr uint32_t kAs5600Sda = PB7;
constexpr uint32_t kAs5600Scl = PB6;
constexpr uint32_t kMotorRpwm = PB8;    // TIM4_CH3
constexpr uint32_t kMotorLpwm = PB9;    // TIM4_CH4
constexpr uint32_t kMotorEnable = PB5;  // R_EN ve L_EN birlikte
#else
constexpr uint32_t kAs5600Sda = 6;
constexpr uint32_t kAs5600Scl = 7;
constexpr uint32_t kMotorRpwm = 0;
constexpr uint32_t kMotorLpwm = 1;
constexpr uint32_t kMotorEnable = 10;
#endif
}  // namespace Pins

namespace As5600 {
constexpr uint8_t kAddress = 0x36;
constexpr uint8_t kStatusRegister = 0x0B;
constexpr uint8_t kRawAngleRegister = 0x0C;
constexpr uint8_t kStatusMagnetDetected = 1U << 5;
constexpr uint8_t kStatusMagnetTooWeak = 1U << 4;
constexpr uint8_t kStatusMagnetTooStrong = 1U << 3;
constexpr uint16_t kDefaultCenterRaw = 1708;
}  // namespace As5600

namespace Motor {
#if defined(ARDUINO_ARCH_STM32)
constexpr uint8_t kRpwmChannel = 3;  // TIM4_CH3 / PB8
constexpr uint8_t kLpwmChannel = 4;  // TIM4_CH4 / PB9
#else
constexpr uint8_t kRpwmChannel = 0;
constexpr uint8_t kLpwmChannel = 1;
#endif
constexpr uint32_t kPwmFrequencyHz = 20000;
constexpr uint8_t kPwmResolutionBits = 8;
constexpr uint8_t kDefaultPwmPercent = 22;
constexpr uint8_t kMinimumPwmPercent = 1;
constexpr uint8_t kMaximumPwmPercent = 50;
constexpr uint32_t kManualDeadTimeMs = 50;
constexpr uint32_t kControlDirectionDeadTimeMs = 2;
constexpr uint32_t kMaximumRunTimeMs = 500;
}  // namespace Motor

namespace PositionControl {
constexpr float kMinimumTargetDegrees = -90.0F;
constexpr float kMaximumTargetDegrees = 90.0F;
constexpr float kMaximumSafePositionDegrees = 100.0F;
constexpr float kToleranceDegrees = 2.0F;
constexpr float kProportionalGain = 0.20F;
constexpr uint8_t kMinimumPwmPercent = 22;
constexpr uint8_t kMaximumPwmPercent = 50;
constexpr uint32_t kMoveTimeoutMs = 3000;
constexpr uint32_t kStallTimeoutMs = 700;
constexpr float kProgressThresholdDegrees = 0.50F;
constexpr float kDirectionCheckTravelDegrees = 2.0F;
constexpr float kDirectionErrorMarginDegrees = 1.0F;
}  // namespace PositionControl

namespace CenteringControl {
constexpr float kCenterDeadbandDegrees = 0.5F;
constexpr float kSoftZoneEndDegrees = 3.0F;
constexpr uint16_t kDefaultRotationDegrees = 900;
constexpr float kVirtualEndstopRampDegrees = 20.0F;
constexpr uint8_t kMinimumEffectivePwmPercent = 22;
constexpr uint8_t kMaximumPdPwmPercent = 30;
constexpr uint8_t kEndstopStartPwmPercent = 30;
constexpr uint8_t kEndstopMaximumPwmPercent = 100;
constexpr float kDefaultKp = 0.40F;
constexpr float kDefaultKd = 0.025F;
constexpr float kMinimumKp = 0.0F;
constexpr float kMaximumKp = 2.0F;
constexpr float kMinimumKd = 0.0F;
constexpr float kMaximumKd = 0.20F;
constexpr float kDefaultSlewRatePercentPerSecond = 80.0F;
constexpr float kMinimumSlewRatePercentPerSecond = 5.0F;
constexpr float kMaximumSlewRatePercentPerSecond = 300.0F;
constexpr float kOutputOffThresholdPercent = 0.5F;
}  // namespace CenteringControl

namespace FfbControl {
constexpr int16_t kMinimumCommandPercent = -100;
constexpr int16_t kMaximumCommandPercent = 100;
constexpr uint32_t kWatchdogTimeoutMs = 100;
}  // namespace FfbControl

constexpr uint32_t kSensorPeriodMs = 10;
constexpr uint32_t kPrintPeriodMs = 100;
constexpr uint32_t kErrorRepeatPeriodMs = 1000;
constexpr float kDegreesPerCount = 360.0F / 4096.0F;
constexpr float kVelocityFilterTimeConstantSeconds = 0.025F;

enum class SensorResult : uint8_t {
  kOk,
  kCommunicationError,
  kMagnetNotDetected,
  kMagnetTooWeak,
  kMagnetTooStrong,
};

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

bool angleInitialized = false;
bool sensorHealthy = false;
uint16_t currentRaw = 0;
uint16_t previousRaw = 0;
int32_t relativeCounts = 0;
float filteredVelocityDegreesPerSecond = 0.0F;
uint32_t previousAngleSampleMs = 0;

uint32_t lastSensorMs = 0;
uint32_t lastPrintMs = 0;
uint32_t lastErrorMs = 0;
SensorResult lastError = SensorResult::kOk;

MotorDirection activeDirection = MotorDirection::kStopped;
MotorDirection pendingDirection = MotorDirection::kStopped;
uint8_t activePwmPercent = 0;
uint8_t pendingPwmPercent = 0;
MotionOwner activeMotionOwner = MotionOwner::kNone;
MotionOwner pendingMotionOwner = MotionOwner::kNone;
bool motorStartPending = false;
uint32_t deadTimeStartedMs = 0;
uint32_t pendingDeadTimeMs = Motor::kManualDeadTimeMs;
uint32_t motorStartedMs = 0;

bool positionControlActive = false;
// Bu mekanikte RPWM yonu AS5600 bagil acisini azaltiyor.
int8_t positionMotorDirectionSign = -1;
float targetAngleDegrees = 0.0F;
float moveStartAngleDegrees = 0.0F;
float moveInitialErrorDegrees = 0.0F;
float lastProgressAngleDegrees = 0.0F;
uint32_t moveStartedMs = 0;
uint32_t lastProgressMs = 0;

bool centeringActive = false;
uint16_t steeringRotationDegrees = CenteringControl::kDefaultRotationDegrees;
float centeringKp = CenteringControl::kDefaultKp;
float centeringKd = CenteringControl::kDefaultKd;
float centeringSlewRatePercentPerSecond =
    CenteringControl::kDefaultSlewRatePercentPerSecond;
float centeringKpTerm = 0.0F;
float centeringKdTerm = 0.0F;
float centeringRequestedOutputPercent = 0.0F;
float centeringAppliedOutputPercent = 0.0F;
uint32_t lastCenteringControlMs = 0;
bool telemetryEnabled = true;

bool ffbControlActive = false;
int16_t ffbCommandPercent = 0;
uint32_t lastFfbCommandMs = 0;

char commandBuffer[32] = {};
uint8_t commandLength = 0;

#if defined(ARDUINO_ARCH_STM32)
HardwareTimer motorPwmTimer(TIM4);
#endif

void writeMotorPwm(uint8_t channel, uint8_t duty) {
#if defined(ARDUINO_ARCH_STM32)
  motorPwmTimer.setCaptureCompare(channel, duty,
                                  RESOLUTION_8B_COMPARE_FORMAT);
#else
  ledcWrite(channel, duty);
#endif
}

bool motorIsBusy() {
  return motorStartPending || activeDirection != MotorDirection::kStopped;
}

void stopMotorOutputs() {
  // Once surucuyu devre disi birak, sonra iki PWM cikisini da sifirla.
  digitalWrite(Pins::kMotorEnable, LOW);
  writeMotorPwm(Motor::kRpwmChannel, 0);
  writeMotorPwm(Motor::kLpwmChannel, 0);

  activeDirection = MotorDirection::kStopped;
  activePwmPercent = 0;
  activeMotionOwner = MotionOwner::kNone;
  motorStartPending = false;
  pendingDirection = MotorDirection::kStopped;
  pendingPwmPercent = 0;
  pendingMotionOwner = MotionOwner::kNone;
  pendingDeadTimeMs = Motor::kManualDeadTimeMs;
}

void initializeMotorOutputs() {
  // PWM zamanlayicisi baglanmadan once pinlerin pasif oldugundan emin ol.
  pinMode(Pins::kMotorRpwm, OUTPUT);
  pinMode(Pins::kMotorLpwm, OUTPUT);
  pinMode(Pins::kMotorEnable, OUTPUT);
  digitalWrite(Pins::kMotorRpwm, LOW);
  digitalWrite(Pins::kMotorLpwm, LOW);
  digitalWrite(Pins::kMotorEnable, LOW);

#if defined(ARDUINO_ARCH_STM32)
  motorPwmTimer.pause();
  motorPwmTimer.setOverflow(Motor::kPwmFrequencyHz, HERTZ_FORMAT);
  motorPwmTimer.setMode(Motor::kRpwmChannel, TIMER_OUTPUT_COMPARE_PWM1,
                        Pins::kMotorRpwm);
  motorPwmTimer.setMode(Motor::kLpwmChannel, TIMER_OUTPUT_COMPARE_PWM1,
                        Pins::kMotorLpwm);
  motorPwmTimer.setCaptureCompare(Motor::kRpwmChannel, 0,
                                  RESOLUTION_8B_COMPARE_FORMAT);
  motorPwmTimer.setCaptureCompare(Motor::kLpwmChannel, 0,
                                  RESOLUTION_8B_COMPARE_FORMAT);
  motorPwmTimer.resume();
#else
  ledcSetup(Motor::kRpwmChannel, Motor::kPwmFrequencyHz,
            Motor::kPwmResolutionBits);
  ledcSetup(Motor::kLpwmChannel, Motor::kPwmFrequencyHz,
            Motor::kPwmResolutionBits);
  ledcAttachPin(Pins::kMotorRpwm, Motor::kRpwmChannel);
  ledcAttachPin(Pins::kMotorLpwm, Motor::kLpwmChannel);
#endif
  stopMotorOutputs();
}

uint8_t pwmPercentToDuty(uint8_t percent) {
  return static_cast<uint8_t>((static_cast<uint16_t>(percent) * 255U + 50U) /
                              100U);
}

void startMotorNow(MotorDirection direction, uint8_t pwmPercent,
                   MotionOwner owner, uint32_t now) {
  const uint8_t duty = pwmPercentToDuty(pwmPercent);

  // Her durumda karsi yon PWM'i sifirda kalir.
  writeMotorPwm(Motor::kRpwmChannel, 0);
  writeMotorPwm(Motor::kLpwmChannel, 0);

  if (direction == MotorDirection::kRight) {
    writeMotorPwm(Motor::kRpwmChannel, duty);
  } else {
    writeMotorPwm(Motor::kLpwmChannel, duty);
  }

  activeDirection = direction;
  activePwmPercent = pwmPercent;
  activeMotionOwner = owner;
  motorStartPending = false;
  pendingDirection = MotorDirection::kStopped;
  pendingPwmPercent = 0;
  pendingMotionOwner = MotionOwner::kNone;
  motorStartedMs = now;
  digitalWrite(Pins::kMotorEnable, HIGH);

  // Yuksek hizli FFB yon degisimlerinde seri cikisi kontrol dongusunu
  // yavaslatmasin. Diger test modlarinda baslangic mesaji korunur.
  if (owner != MotionOwner::kFfb) {
    Serial.print("Motor calisiyor: ");
    Serial.print(direction == MotorDirection::kRight ? "SAG" : "SOL");
    Serial.print(" | PWM: %");
    Serial.println(pwmPercent);
  }
}

void requestMotorStart(MotorDirection direction, uint8_t pwmPercent,
                       uint32_t now) {
  positionControlActive = false;
  centeringActive = false;
  ffbControlActive = false;
  ffbCommandPercent = 0;
  if (!angleInitialized || !sensorHealthy) {
    stopMotorOutputs();
    Serial.println("HATA: Gecerli AS5600 verisi olmadan motor calistirilamaz");
    return;
  }

  // Ayni yonde yeniden komut verilse bile once guvenli olu zaman uygula.
  stopMotorOutputs();
  pendingDirection = direction;
  pendingPwmPercent = pwmPercent;
  pendingMotionOwner = MotionOwner::kManual;
  pendingDeadTimeMs = Motor::kManualDeadTimeMs;
  deadTimeStartedMs = now;
  motorStartPending = true;

  Serial.print("Motor komutu alindi: ");
  Serial.print(direction == MotorDirection::kRight ? "SAG" : "SOL");
  Serial.print(" | PWM: %");
  Serial.print(pwmPercent);
  Serial.println(" | 50 ms guvenli bekleme");
}

void commandContinuousMotor(MotorDirection direction, uint8_t pwmPercent,
                            MotionOwner owner, uint32_t now) {
  if (activeDirection == direction &&
      activeMotionOwner == owner) {
    const uint8_t duty = pwmPercentToDuty(pwmPercent);
    if (direction == MotorDirection::kRight) {
      writeMotorPwm(Motor::kRpwmChannel, duty);
    } else {
      writeMotorPwm(Motor::kLpwmChannel, duty);
    }
    activePwmPercent = pwmPercent;
    return;
  }

  if (motorStartPending && pendingDirection == direction &&
      pendingMotionOwner == owner) {
    pendingPwmPercent = pwmPercent;
    return;
  }

  // Yon degisiminde cikislari kapat ve olu zamani yeniden baslat.
  stopMotorOutputs();
  pendingDirection = direction;
  pendingPwmPercent = pwmPercent;
  pendingMotionOwner = owner;
  pendingDeadTimeMs = Motor::kControlDirectionDeadTimeMs;
  deadTimeStartedMs = now;
  motorStartPending = true;
}

void serviceMotor(uint32_t now) {
  if (motorStartPending && now - deadTimeStartedMs >= pendingDeadTimeMs) {
    if (!sensorHealthy) {
      stopMotorOutputs();
      positionControlActive = false;
      centeringActive = false;
      ffbControlActive = false;
      ffbCommandPercent = 0;
      Serial.println("HATA: Sensor hatasi nedeniyle motor baslatilmadi");
    } else {
      startMotorNow(pendingDirection, pendingPwmPercent, pendingMotionOwner,
                    now);
    }
  }

  if (activeMotionOwner == MotionOwner::kManual &&
      now - motorStartedMs >= Motor::kMaximumRunTimeMs) {
    stopMotorOutputs();
    Serial.println("Motor otomatik durduruldu: 500 ms zaman asimi");
  }
}

bool isAs5600Connected() {
  Wire.beginTransmission(As5600::kAddress);
  return Wire.endTransmission() == 0;
}

bool readRegisters(uint8_t firstRegister, uint8_t* destination,
                   uint8_t length) {
  Wire.beginTransmission(As5600::kAddress);
  Wire.write(firstRegister);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const uint8_t received = Wire.requestFrom(As5600::kAddress, length);
  if (received != length) {
    while (Wire.available()) {
      Wire.read();
    }
    return false;
  }

  for (uint8_t index = 0; index < length; ++index) {
    if (!Wire.available()) {
      return false;
    }
    destination[index] = static_cast<uint8_t>(Wire.read());
  }
  return true;
}

SensorResult readAs5600(uint16_t& rawAngle) {
  uint8_t status = 0;
  if (!readRegisters(As5600::kStatusRegister, &status, 1)) {
    return SensorResult::kCommunicationError;
  }
  if ((status & As5600::kStatusMagnetDetected) == 0) {
    return SensorResult::kMagnetNotDetected;
  }
  if ((status & As5600::kStatusMagnetTooWeak) != 0) {
    return SensorResult::kMagnetTooWeak;
  }
  if ((status & As5600::kStatusMagnetTooStrong) != 0) {
    return SensorResult::kMagnetTooStrong;
  }

  uint8_t rawBytes[2] = {0, 0};
  if (!readRegisters(As5600::kRawAngleRegister, rawBytes,
                     sizeof(rawBytes))) {
    return SensorResult::kCommunicationError;
  }

  rawAngle = (static_cast<uint16_t>(rawBytes[0] & 0x0F) << 8) |
             static_cast<uint16_t>(rawBytes[1]);
  return SensorResult::kOk;
}

const char* sensorErrorMessage(SensorResult result) {
  switch (result) {
    case SensorResult::kCommunicationError:
      return "HATA: AS5600 ile I2C haberlesmesi kurulamadi";
    case SensorResult::kMagnetNotDetected:
      return "HATA: AS5600 miknatis algilamiyor";
    case SensorResult::kMagnetTooWeak:
      return "HATA: AS5600 miknatis alani cok zayif";
    case SensorResult::kMagnetTooStrong:
      return "HATA: AS5600 miknatis alani cok guclu";
    case SensorResult::kOk:
      break;
  }
  return "";
}

void reportSensorError(SensorResult result, uint32_t now) {
  const bool motorWasBusy = motorIsBusy();
  positionControlActive = false;
  centeringActive = false;
  ffbControlActive = false;
  ffbCommandPercent = 0;
  stopMotorOutputs();
  if (motorWasBusy) {
    Serial.println("Motor sensor guvenligi nedeniyle durduruldu");
  }

  if (result != lastError || now - lastErrorMs >= kErrorRepeatPeriodMs) {
    Serial.println(sensorErrorMessage(result));
    lastErrorMs = now;
  }
  lastError = result;
  sensorHealthy = false;
  filteredVelocityDegreesPerSecond = 0.0F;
}

void updateAngle(uint32_t now) {
  uint16_t newRaw = 0;
  const SensorResult result = readAs5600(newRaw);
  if (result != SensorResult::kOk) {
    // Hatali okuma son gecerli aci verilerini degistirmez.
    reportSensorError(result, now);
    return;
  }

  if (!sensorHealthy && lastError != SensorResult::kOk) {
    Serial.println("AS5600 Baglantisi Yeniden Kuruldu");
  }
  sensorHealthy = true;
  lastError = SensorResult::kOk;

  if (!angleInitialized) {
    currentRaw = newRaw;
    previousRaw = newRaw;
    int32_t defaultCenterDelta =
        static_cast<int32_t>(newRaw) - As5600::kDefaultCenterRaw;
    if (defaultCenterDelta > 2048) {
      defaultCenterDelta -= 4096;
    } else if (defaultCenterDelta < -2048) {
      defaultCenterDelta += 4096;
    }
    relativeCounts = defaultCenterDelta;
    filteredVelocityDegreesPerSecond = 0.0F;
    previousAngleSampleMs = now;
    angleInitialized = true;
    Serial.print("Varsayilan Merkez RAW: ");
    Serial.print(As5600::kDefaultCenterRaw);
    Serial.print(" | Baslangic Bagil Aci: ");
    Serial.print(relativeCounts * kDegreesPerCount, 2);
    Serial.println(" derece");
    Serial.println();
    return;
  }

  int32_t delta = static_cast<int32_t>(newRaw) -
                  static_cast<int32_t>(previousRaw);
  if (delta > 2048) {
    delta -= 4096;
  } else if (delta < -2048) {
    delta += 4096;
  }

  const uint32_t elapsedMs = now - previousAngleSampleMs;
  if (elapsedMs > 0) {
    const float elapsedSeconds = elapsedMs * 0.001F;
    const float instantaneousVelocity =
        delta * kDegreesPerCount / elapsedSeconds;
    const float velocityFilterFactor =
        elapsedSeconds /
        (kVelocityFilterTimeConstantSeconds + elapsedSeconds);
    filteredVelocityDegreesPerSecond +=
        velocityFilterFactor *
        (instantaneousVelocity - filteredVelocityDegreesPerSecond);
  }
  previousAngleSampleMs = now;

  relativeCounts += delta;
  previousRaw = newRaw;
  currentRaw = newRaw;
}

float currentRelativeAngleDegrees() {
  return relativeCounts * kDegreesPerCount;
}

void stopAllMotion() {
  positionControlActive = false;
  centeringActive = false;
  ffbControlActive = false;
  ffbCommandPercent = 0;
  centeringKpTerm = 0.0F;
  centeringKdTerm = 0.0F;
  centeringRequestedOutputPercent = 0.0F;
  centeringAppliedOutputPercent = 0.0F;
  lastCenteringControlMs = 0;
  stopMotorOutputs();
}

void abortPositionMove(const char* reason) {
  stopAllMotion();
  Serial.print("HATA: Hedef hareketi iptal edildi: ");
  Serial.println(reason);
}

void startPositionMove(float requestedAngle, uint32_t now) {
  if (!angleInitialized || !sensorHealthy) {
    stopAllMotion();
    Serial.println("HATA: Gecerli AS5600 verisi olmadan hedefe gidilemez");
    return;
  }
  if (!isfinite(requestedAngle) ||
      requestedAngle < PositionControl::kMinimumTargetDegrees ||
      requestedAngle > PositionControl::kMaximumTargetDegrees) {
    Serial.println("HATA: Hedef -90 ile +90 derece arasinda olmali");
    return;
  }

  const float currentAngle = currentRelativeAngleDegrees();
  if (fabsf(currentAngle) >
      PositionControl::kMaximumSafePositionDegrees) {
    stopAllMotion();
    Serial.println("HATA: Mevcut konum guvenli aci sinirinin disinda");
    return;
  }

  stopAllMotion();
  targetAngleDegrees = requestedAngle;
  moveStartAngleDegrees = currentAngle;
  moveInitialErrorDegrees = fabsf(targetAngleDegrees - currentAngle);
  lastProgressAngleDegrees = currentAngle;
  moveStartedMs = now;
  lastProgressMs = now;

  Serial.print("Hedef konum: ");
  Serial.print(targetAngleDegrees, 2);
  Serial.println(" derece");

  if (moveInitialErrorDegrees <= PositionControl::kToleranceDegrees) {
    Serial.println("Direksiyon zaten hedef toleransi icinde");
    return;
  }

  positionControlActive = true;
}

void updatePositionController(uint32_t now) {
  if (!positionControlActive) {
    return;
  }
  if (!sensorHealthy) {
    abortPositionMove("sensor verisi gecersiz");
    return;
  }

  const float currentAngle = currentRelativeAngleDegrees();
  const float error = targetAngleDegrees - currentAngle;
  const float absoluteError = fabsf(error);

  if (fabsf(currentAngle) >
      PositionControl::kMaximumSafePositionDegrees) {
    abortPositionMove("100 derece yazilimsal konum siniri");
    return;
  }

  if (absoluteError <= PositionControl::kToleranceDegrees) {
    stopAllMotion();
    Serial.print("Hedefe ulasildi | Aci: ");
    Serial.println(currentAngle, 2);
    return;
  }

  if (now - moveStartedMs >= PositionControl::kMoveTimeoutMs) {
    abortPositionMove("3 saniye zaman asimi");
    return;
  }

  if (fabsf(currentAngle - lastProgressAngleDegrees) >=
      PositionControl::kProgressThresholdDegrees) {
    lastProgressAngleDegrees = currentAngle;
    lastProgressMs = now;
  } else if (now - lastProgressMs >= PositionControl::kStallTimeoutMs) {
    abortPositionMove("motor hareket etmiyor");
    return;
  }

  const float travelled = fabsf(currentAngle - moveStartAngleDegrees);
  if (travelled >= PositionControl::kDirectionCheckTravelDegrees &&
      absoluteError > moveInitialErrorDegrees +
                          PositionControl::kDirectionErrorMarginDegrees) {
    abortPositionMove("motor-enkoder yonu ters; 'd' komutunu kullanin");
    return;
  }

  float requestedPwm = PositionControl::kMinimumPwmPercent +
                       PositionControl::kProportionalGain * absoluteError;
  if (requestedPwm > PositionControl::kMaximumPwmPercent) {
    requestedPwm = PositionControl::kMaximumPwmPercent;
  }
  const uint8_t pwmPercent = static_cast<uint8_t>(requestedPwm + 0.5F);

  int8_t directionValue = error > 0.0F ? 1 : -1;
  directionValue *= positionMotorDirectionSign;
  commandContinuousMotor(directionValue > 0 ? MotorDirection::kRight
                                            : MotorDirection::kLeft,
                         pwmPercent, MotionOwner::kPositionControl, now);
}

void startCentering() {
  if (!angleInitialized || !sensorHealthy) {
    stopAllMotion();
    Serial.println("HATA: Gecerli AS5600 verisi olmadan merkezleme acilamaz");
    return;
  }
  stopAllMotion();
  filteredVelocityDegreesPerSecond = 0.0F;
  lastCenteringControlMs = millis();
  centeringActive = true;
  Serial.print("Aktif merkezleme acildi | Mod: ");
  Serial.print(steeringRotationDegrees);
  Serial.print(" derece | Sanal sinir: +/-");
  Serial.print(steeringRotationDegrees / 2);
  Serial.println(" derece");
  Serial.print("PD: Kp=");
  Serial.print(centeringKp, 3);
  Serial.print(" | Kd=");
  Serial.print(centeringKd, 3);
  Serial.print(" | Normal cikis en fazla: %");
  Serial.println(CenteringControl::kMaximumPdPwmPercent);
}

void abortCentering(const char* reason) {
  centeringActive = false;
  centeringKpTerm = 0.0F;
  centeringKdTerm = 0.0F;
  centeringRequestedOutputPercent = 0.0F;
  centeringAppliedOutputPercent = 0.0F;
  lastCenteringControlMs = 0;
  stopMotorOutputs();
  Serial.print("HATA: Aktif merkezleme kapatildi: ");
  Serial.println(reason);
}

float smoothstep(float value) {
  value = constrain(value, 0.0F, 1.0F);
  return value * value * (3.0F - 2.0F * value);
}

float applyCenteringSlewRate(float requestedOutputPercent, uint32_t now) {
  if (lastCenteringControlMs == 0) {
    lastCenteringControlMs = now;
    centeringAppliedOutputPercent = 0.0F;
    return centeringAppliedOutputPercent;
  }

  const uint32_t elapsedMs = now - lastCenteringControlMs;
  if (elapsedMs == 0) {
    return centeringAppliedOutputPercent;
  }
  lastCenteringControlMs = now;

  const float maximumChange =
      centeringSlewRatePercentPerSecond * elapsedMs * 0.001F;
  const float difference =
      requestedOutputPercent - centeringAppliedOutputPercent;
  centeringAppliedOutputPercent +=
      constrain(difference, -maximumChange, maximumChange);
  return centeringAppliedOutputPercent;
}

void updateCenteringController(uint32_t now) {
  if (!centeringActive) {
    return;
  }
  if (!sensorHealthy) {
    abortCentering("sensor verisi gecersiz");
    return;
  }

  const float angle = currentRelativeAngleDegrees();
  const float absoluteAngle = fabsf(angle);

  // Yay terimini merkezde yumusat. Damping terimi yumusatilmaz; boylece
  // direksiyon merkeze hizli yaklasirken motor hareketi frenleyebilir.
  float springScale = 1.0F;
  if (absoluteAngle <= CenteringControl::kCenterDeadbandDegrees) {
    springScale = 0.0F;
  } else if (absoluteAngle < CenteringControl::kSoftZoneEndDegrees) {
    const float softZonePosition =
        (absoluteAngle - CenteringControl::kCenterDeadbandDegrees) /
        (CenteringControl::kSoftZoneEndDegrees -
         CenteringControl::kCenterDeadbandDegrees);
    springScale = smoothstep(softZonePosition);
  }

  centeringKpTerm = centeringKp * angle * springScale;
  centeringKdTerm = centeringKd * filteredVelocityDegreesPerSecond;
  float requestedOutput = -(centeringKpTerm + centeringKdTerm);
  requestedOutput = constrain(
      requestedOutput,
      -static_cast<float>(CenteringControl::kMaximumPdPwmPercent),
      static_cast<float>(CenteringControl::kMaximumPdPwmPercent));

  // RS-775 bu mekanikte %22 altinda hareket etmiyor. Merkezde sabit bir
  // minimum uygulamak yerine motorun olu bolge telafisini ayni smoothstep
  // katsayisiyla 0'dan %22'ye cikar. Boylece 0.5 derece icinde motor kapali,
  // 0.5-3 derece arasinda gecis yumusak ve 3 derece disinda komut etkilidir.
  if (fabsf(requestedOutput) >=
      CenteringControl::kOutputOffThresholdPercent) {
    const float minimumEffectiveOutput =
        CenteringControl::kMinimumEffectivePwmPercent * springScale;
    if (fabsf(requestedOutput) < minimumEffectiveOutput) {
      requestedOutput =
          requestedOutput > 0.0F ? minimumEffectiveOutput
                                 : -minimumEffectiveOutput;
    }
  }

  // Sanal son durak PD merkezlemeden ayridir. Son 20 derecede merkeze dogru
  // %30-%100 kuvvet uygular ve disari yonlu bir damping istegine yenilmez.
  const float virtualLimitDegrees = steeringRotationDegrees * 0.5F;
  const float virtualEndstopStartDegrees =
      virtualLimitDegrees - CenteringControl::kVirtualEndstopRampDegrees;
  if (absoluteAngle > virtualEndstopStartDegrees) {
    const float endstopPosition =
        (absoluteAngle - virtualEndstopStartDegrees) /
        CenteringControl::kVirtualEndstopRampDegrees;
    const float endstopScale = smoothstep(endstopPosition);
    const float endstopMagnitude =
        CenteringControl::kEndstopStartPwmPercent +
        (CenteringControl::kEndstopMaximumPwmPercent -
         CenteringControl::kEndstopStartPwmPercent) *
            endstopScale;
    requestedOutput = angle > 0.0F ? -endstopMagnitude : endstopMagnitude;
  }

  if (fabsf(requestedOutput) <
      CenteringControl::kOutputOffThresholdPercent) {
    requestedOutput = 0.0F;
  }
  centeringRequestedOutputPercent = requestedOutput;

  // Motorun kullanilabilir ilk noktasi %22 oldugu icin sifira inme ve yon
  // degisimi geciktirilmez. Once cikis tamamen kesilir; yeni yon bir sonraki
  // adimda surucunun guvenli gecis mekanizmasiyla baslatilir.
  if (requestedOutput == 0.0F) {
    centeringAppliedOutputPercent = 0.0F;
    lastCenteringControlMs = now;
    stopMotorOutputs();
    return;
  }
  if (centeringAppliedOutputPercent != 0.0F &&
      ((requestedOutput > 0.0F) !=
       (centeringAppliedOutputPercent > 0.0F))) {
    centeringAppliedOutputPercent = 0.0F;
    lastCenteringControlMs = now;
    stopMotorOutputs();
    return;
  }

  const float appliedOutput =
      applyCenteringSlewRate(requestedOutput, now);

  if (fabsf(appliedOutput) <
      CenteringControl::kOutputOffThresholdPercent) {
    stopMotorOutputs();
    return;
  }

  int8_t directionValue = appliedOutput > 0.0F ? 1 : -1;
  directionValue *= positionMotorDirectionSign;
  uint8_t pwmPercent =
      static_cast<uint8_t>(fabsf(appliedOutput) + 0.5F);
  if (pwmPercent < CenteringControl::kMinimumEffectivePwmPercent) {
    pwmPercent = CenteringControl::kMinimumEffectivePwmPercent;
  }
  commandContinuousMotor(directionValue > 0 ? MotorDirection::kRight
                                            : MotorDirection::kLeft,
                         pwmPercent, MotionOwner::kCentering, now);
}

void setFfbTorqueCommand(int16_t commandPercent, uint32_t now) {
  if (!angleInitialized || !sensorHealthy) {
    stopAllMotion();
    Serial.println("HATA: Gecerli AS5600 verisi olmadan FFB calistirilamaz");
    return;
  }

  if (commandPercent == 0 && !ffbControlActive) {
    stopMotorOutputs();
    return;
  }

  if (!ffbControlActive) {
    stopAllMotion();
    ffbControlActive = true;
    Serial.println("FFB seri tork modu aktif | Aralik: -100 ... +100");
  }
  ffbCommandPercent = commandPercent;
  lastFfbCommandMs = now;
}

void updateFfbController(uint32_t now) {
  if (!ffbControlActive) {
    return;
  }
  if (!sensorHealthy) {
    stopAllMotion();
    Serial.println("HATA: Sensor hatasi nedeniyle FFB kapatildi");
    return;
  }
  if (now - lastFfbCommandMs >= FfbControl::kWatchdogTimeoutMs) {
    stopAllMotion();
    Serial.println("HATA: FFB watchdog zaman asimi; motor kapatildi");
    return;
  }

  float requestedOutput = static_cast<float>(ffbCommandPercent);

  // Sanal son durak FFB modunda da en yuksek oncelige sahiptir. Oyun disari
  // tork istese bile son 20 derecede merkeze dogru %30-%100 uygulanir.
  const float angle = currentRelativeAngleDegrees();
  const float absoluteAngle = fabsf(angle);
  const float virtualLimitDegrees = steeringRotationDegrees * 0.5F;
  const float virtualEndstopStartDegrees =
      virtualLimitDegrees - CenteringControl::kVirtualEndstopRampDegrees;
  if (absoluteAngle > virtualEndstopStartDegrees) {
    const float endstopPosition =
        (absoluteAngle - virtualEndstopStartDegrees) /
        CenteringControl::kVirtualEndstopRampDegrees;
    const float endstopScale = smoothstep(endstopPosition);
    const float endstopMagnitude =
        CenteringControl::kEndstopStartPwmPercent +
        (CenteringControl::kEndstopMaximumPwmPercent -
         CenteringControl::kEndstopStartPwmPercent) *
            endstopScale;
    requestedOutput = angle > 0.0F ? -endstopMagnitude : endstopMagnitude;
  }

  if (fabsf(requestedOutput) < 0.5F) {
    stopMotorOutputs();
    return;
  }

  int8_t directionValue = requestedOutput > 0.0F ? 1 : -1;
  directionValue *= positionMotorDirectionSign;
  const uint8_t pwmPercent =
      static_cast<uint8_t>(fabsf(requestedOutput) + 0.5F);
  commandContinuousMotor(directionValue > 0 ? MotorDirection::kRight
                                            : MotorDirection::kLeft,
                         pwmPercent, MotionOwner::kFfb, now);
}

void zeroCurrentPosition() {
  if (motorIsBusy() || positionControlActive || centeringActive ||
      ffbControlActive) {
    Serial.println("HATA: Sifirlamadan once motoru durdurun");
    return;
  }
  if (!angleInitialized || !sensorHealthy) {
    Serial.println("HATA: Sifirlama icin gecerli sensor verisi yok");
    return;
  }

  relativeCounts = 0;
  previousRaw = currentRaw;
  filteredVelocityDegreesPerSecond = 0.0F;
  Serial.println("Sifir Konumu Guncellendi");
}

void startCenteringWithDefaults() {
  if (!angleInitialized || !sensorHealthy) {
    stopAllMotion();
    Serial.println("HATA: Varsayilan baslatma icin AS5600 verisi gecersiz");
    return;
  }

  // Kullanici bu komuttan once direksiyonu fiziksel merkeze getirir. Motor
  // once tamamen kapatilir, sonra mevcut konum yeni sifir kabul edilir.
  stopAllMotion();
  relativeCounts = 0;
  previousRaw = currentRaw;
  filteredVelocityDegreesPerSecond = 0.0F;

  steeringRotationDegrees = CenteringControl::kDefaultRotationDegrees;
  centeringKp = CenteringControl::kDefaultKp;
  centeringKd = CenteringControl::kDefaultKd;
  centeringSlewRatePercentPerSecond =
      CenteringControl::kDefaultSlewRatePercentPerSecond;
  positionMotorDirectionSign = -1;
  telemetryEnabled = true;

  Serial.println("Varsayilan ayarlar yuklendi:");
  Serial.print("  Merkez RAW: ");
  Serial.print(currentRaw);
  Serial.print(" | Mod: ");
  Serial.print(steeringRotationDegrees);
  Serial.print(" | Kp: ");
  Serial.print(centeringKp, 3);
  Serial.print(" | Kd: ");
  Serial.print(centeringKd, 3);
  Serial.print(" | Slew-rate: %");
  Serial.print(centeringSlewRatePercentPerSecond, 1);
  Serial.println("/s");

  startCentering();
}

void printHelp() {
  Serial.println("Komutlar:");
  Serial.println("  a         Merkezde tek komutla varsayilan ayarlari baslat");
  Serial.println("  c on/off  Aktif merkezlemeyi ac/kapat");
  Serial.println("  kp <0-2>  PD oransal kazanci");
  Serial.println("  kd <0-.2> PD turev kazanci");
  Serial.println("  rt <5-300> PWM/saniye slew-rate siniri");
  Serial.println("  t on/off  100 ms kontrol telemetrisi");
  Serial.println("  f <-100..100> Isaretli FFB tork komutu");
  Serial.println("  m [mod]   Modu goster veya 360/900/1080 derece sec");
  Serial.println("  g <aci>   -90 ... +90 derece hedef konum");
  Serial.println("  d         Kontrol motor yonunu tersle");
  Serial.println("  r [1-50]  Sag yon, varsayilan %22 PWM");
  Serial.println("  l [1-50]  Sol yon, varsayilan %22 PWM");
  Serial.println("  s         Motoru hemen durdur");
  Serial.println("  z         Mevcut konumu sifirla");
  Serial.println("  h         Bu yardimi goster");
}

char* skipSpaces(char* text) {
  while (*text != '\0' && isspace(static_cast<unsigned char>(*text))) {
    ++text;
  }
  return text;
}

bool hasOnlyTrailingSpaces(char* text) {
  return *skipSpaces(text) == '\0';
}

bool matchesTwoLetterCommand(const char* text, char first, char second) {
  return tolower(static_cast<unsigned char>(text[0])) == first &&
         tolower(static_cast<unsigned char>(text[1])) == second &&
         (text[2] == '\0' ||
          isspace(static_cast<unsigned char>(text[2])));
}

bool parseFloatArgument(char* argument, float& value) {
  argument = skipSpaces(argument);
  if (*argument == '\0') {
    return false;
  }
  char* end = nullptr;
  value = strtof(argument, &end);
  return end != argument && hasOnlyTrailingSpaces(end) && isfinite(value);
}

void handleCommand(char* line, uint32_t now) {
  char* cursor = skipSpaces(line);
  if (*cursor == '\0') {
    return;
  }

  if (matchesTwoLetterCommand(cursor, 'k', 'p')) {
    float requestedKp = 0.0F;
    if (!parseFloatArgument(cursor + 2, requestedKp) ||
        requestedKp < CenteringControl::kMinimumKp ||
        requestedKp > CenteringControl::kMaximumKp) {
      Serial.println("HATA: Kp 0.0 ile 2.0 arasinda olmali");
      return;
    }
    centeringKp = requestedKp;
    Serial.print("Kp ayarlandi: ");
    Serial.println(centeringKp, 3);
    return;
  }

  if (matchesTwoLetterCommand(cursor, 'k', 'd')) {
    float requestedKd = 0.0F;
    if (!parseFloatArgument(cursor + 2, requestedKd) ||
        requestedKd < CenteringControl::kMinimumKd ||
        requestedKd > CenteringControl::kMaximumKd) {
      Serial.println("HATA: Kd 0.0 ile 0.2 arasinda olmali");
      return;
    }
    centeringKd = requestedKd;
    Serial.print("Kd ayarlandi: ");
    Serial.println(centeringKd, 3);
    return;
  }

  if (matchesTwoLetterCommand(cursor, 'r', 't')) {
    float requestedSlewRate = 0.0F;
    if (!parseFloatArgument(cursor + 2, requestedSlewRate) ||
        requestedSlewRate <
            CenteringControl::kMinimumSlewRatePercentPerSecond ||
        requestedSlewRate >
            CenteringControl::kMaximumSlewRatePercentPerSecond) {
      Serial.println("HATA: Slew-rate 5 ile 300 PWM yuzdesi/saniye olmali");
      return;
    }
    centeringSlewRatePercentPerSecond = requestedSlewRate;
    Serial.print("Slew-rate ayarlandi: %");
    Serial.print(centeringSlewRatePercentPerSecond, 1);
    Serial.println("/s");
    return;
  }

  const char command = static_cast<char>(
      tolower(static_cast<unsigned char>(*cursor++)));
  if (*cursor != '\0' && !isspace(static_cast<unsigned char>(*cursor))) {
    Serial.println("Gecersiz komut. Yardim icin 'h' gonderin.");
    return;
  }
  cursor = skipSpaces(cursor);

  if (command == 's' && hasOnlyTrailingSpaces(cursor)) {
    const bool wasBusy =
        motorIsBusy() || positionControlActive || centeringActive ||
        ffbControlActive;
    stopAllMotion();
    Serial.println(wasBusy ? "Motor durduruldu" : "Motor zaten duruyor");
    return;
  }
  if (command == 'a' && hasOnlyTrailingSpaces(cursor)) {
    startCenteringWithDefaults();
    return;
  }
  if (command == 'z' && hasOnlyTrailingSpaces(cursor)) {
    zeroCurrentPosition();
    return;
  }
  if (command == 'h' && hasOnlyTrailingSpaces(cursor)) {
    printHelp();
    return;
  }
  if (command == 't') {
    const bool enableRequested =
        tolower(static_cast<unsigned char>(cursor[0])) == 'o' &&
        tolower(static_cast<unsigned char>(cursor[1])) == 'n' &&
        hasOnlyTrailingSpaces(cursor + 2);
    const bool disableRequested =
        tolower(static_cast<unsigned char>(cursor[0])) == 'o' &&
        tolower(static_cast<unsigned char>(cursor[1])) == 'f' &&
        tolower(static_cast<unsigned char>(cursor[2])) == 'f' &&
        hasOnlyTrailingSpaces(cursor + 3);
    if (enableRequested || disableRequested) {
      telemetryEnabled = enableRequested;
      Serial.println(telemetryEnabled ? "Telemetri acildi"
                                      : "Telemetri kapatildi");
      return;
    }
    Serial.println("HATA: Telemetri komutu 't on' veya 't off' olmali");
    return;
  }
  if (command == 'c') {
    const bool enableRequested =
        *cursor == '\0' ||
        (tolower(static_cast<unsigned char>(cursor[0])) == 'o' &&
         tolower(static_cast<unsigned char>(cursor[1])) == 'n' &&
         hasOnlyTrailingSpaces(cursor + 2));
    const bool disableRequested =
        tolower(static_cast<unsigned char>(cursor[0])) == 'o' &&
        tolower(static_cast<unsigned char>(cursor[1])) == 'f' &&
        tolower(static_cast<unsigned char>(cursor[2])) == 'f' &&
        hasOnlyTrailingSpaces(cursor + 3);
    if (enableRequested) {
      startCentering();
      return;
    }
    if (disableRequested) {
      const bool wasActive = centeringActive;
      stopAllMotion();
      Serial.println(wasActive ? "Aktif merkezleme kapatildi"
                               : "Aktif merkezleme zaten kapali");
      return;
    }
    Serial.println("HATA: Merkezleme komutu 'c on' veya 'c off' olmali");
    return;
  }
  if (command == 'm') {
    if (*cursor == '\0') {
      Serial.print("Direksiyon modu: ");
      Serial.print(steeringRotationDegrees);
      Serial.print(" derece | Sanal sinir: +/-");
      Serial.print(steeringRotationDegrees / 2);
      Serial.println(" derece");
      return;
    }

    char* end = nullptr;
    const long requestedMode = strtol(cursor, &end, 10);
    if (end == cursor || !hasOnlyTrailingSpaces(end) ||
        (requestedMode != 360 && requestedMode != 900 &&
         requestedMode != 1080)) {
      Serial.println("HATA: Mod 360, 900 veya 1080 olmali");
      return;
    }

    stopAllMotion();
    steeringRotationDegrees = static_cast<uint16_t>(requestedMode);
    Serial.print("Direksiyon modu ayarlandi: ");
    Serial.print(steeringRotationDegrees);
    Serial.print(" derece | Sanal sinir: +/-");
    Serial.print(steeringRotationDegrees / 2);
    Serial.println(" derece");
    Serial.println("Merkezlemeyi baslatmak icin 'c on' gonderin");
    return;
  }
  if (command == 'd' && hasOnlyTrailingSpaces(cursor)) {
    if (motorIsBusy() || positionControlActive || centeringActive ||
        ffbControlActive) {
      Serial.println("HATA: Yonu degistirmeden once motoru durdurun");
      return;
    }
    positionMotorDirectionSign *= -1;
    Serial.print("Hedef kontrol motor yonu: ");
    Serial.println(positionMotorDirectionSign > 0 ? "NORMAL" : "TERS");
    return;
  }

  if (command == 'g') {
    if (*cursor == '\0') {
      Serial.println("HATA: Ornek hedef komutu: g 10");
      return;
    }
    char* end = nullptr;
    const float requestedAngle = strtof(cursor, &end);
    if (end == cursor || !hasOnlyTrailingSpaces(end) ||
        !isfinite(requestedAngle)) {
      Serial.println("HATA: Hedef aci sayi olmali");
      return;
    }
    startPositionMove(requestedAngle, now);
    return;
  }

  if (command == 'f') {
    char* end = nullptr;
    const long requestedFfb = strtol(cursor, &end, 10);
    if (end == cursor || !hasOnlyTrailingSpaces(end) ||
        requestedFfb < FfbControl::kMinimumCommandPercent ||
        requestedFfb > FfbControl::kMaximumCommandPercent) {
      if (ffbControlActive) {
        stopAllMotion();
      }
      Serial.println("HATA: FFB komutu -100 ile +100 arasinda olmali");
      return;
    }
    setFfbTorqueCommand(static_cast<int16_t>(requestedFfb), now);
    return;
  }

  if (command == 'r' || command == 'l') {
    long pwmPercent = Motor::kDefaultPwmPercent;
    if (*cursor != '\0') {
      char* end = nullptr;
      pwmPercent = strtol(cursor, &end, 10);
      if (end == cursor || !hasOnlyTrailingSpaces(end)) {
        Serial.println("HATA: PWM tam sayi olmali");
        return;
      }
    }

    if (pwmPercent < Motor::kMinimumPwmPercent ||
        pwmPercent > Motor::kMaximumPwmPercent) {
      Serial.println("HATA: PWM yuzdesi 1 ile 50 arasinda olmali");
      return;
    }

    requestMotorStart(command == 'r' ? MotorDirection::kRight
                                     : MotorDirection::kLeft,
                      static_cast<uint8_t>(pwmPercent), now);
    return;
  }

  Serial.println("Gecersiz komut. Yardim icin 'h' gonderin.");
}

void readSerialCommands(uint32_t now) {
  while (Serial.available() > 0) {
    const char received = static_cast<char>(Serial.read());

    // Acil durdurma satir sonunu beklemeden, tek karakter gelir gelmez calisir.
    if (commandLength == 0 && (received == 's' || received == 'S')) {
      const bool wasBusy =
          motorIsBusy() || positionControlActive || centeringActive ||
          ffbControlActive;
      stopAllMotion();
      Serial.println(wasBusy ? "Motor durduruldu" : "Motor zaten duruyor");
    } else if (received == '\r' || received == '\n') {
      if (commandLength > 0) {
        commandBuffer[commandLength] = '\0';
        handleCommand(commandBuffer, now);
        commandLength = 0;
      }
    } else if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = received;
    } else {
      commandLength = 0;
      Serial.println("HATA: Komut cok uzun");
    }
  }
}

void printStatus() {
  if (!telemetryEnabled || !angleInitialized || !sensorHealthy) {
    return;
  }

  Serial.print("ANGLE: ");
  Serial.print(currentRelativeAngleDegrees(), 2);
  Serial.print(" | VELOCITY: ");
  Serial.print(filteredVelocityDegreesPerSecond, 1);
  Serial.print(" | KP_TERM: ");
  Serial.print(centeringActive ? centeringKpTerm : 0.0F, 2);
  Serial.print(" | KD_TERM: ");
  Serial.print(centeringActive ? centeringKdTerm : 0.0F, 2);
  Serial.print(" | PWM: ");
  Serial.print(activePwmPercent);
  Serial.print(" | DIRECTION: ");

  if (motorStartPending) {
    Serial.print("WAIT");
  } else if (activeDirection == MotorDirection::kRight) {
    Serial.print("RIGHT");
  } else if (activeDirection == MotorDirection::kLeft) {
    Serial.print("LEFT");
  } else {
    Serial.print("OFF");
  }
  Serial.print(" | RAW: ");
  Serial.print(currentRaw);
  Serial.print(" | ABSOLUTE: ");
  Serial.println(currentRaw * kDegreesPerCount, 2);
}

void setup() {
  initializeMotorOutputs();

  Serial.begin(115200);
#if defined(ARDUINO_ARCH_STM32)
  Wire.setSDA(Pins::kAs5600Sda);
  Wire.setSCL(Pins::kAs5600Scl);
  Wire.begin();
#else
  Wire.begin(Pins::kAs5600Sda, Pins::kAs5600Scl);
#endif
  Wire.setClock(100000);
#if !defined(ARDUINO_ARCH_STM32)
  Wire.setTimeOut(20);
#endif

  Serial.println();
  Serial.println("AS5600 Konum Kontrolu ve Aktif Merkezleme Baslatiliyor...");
  Serial.println("Hedef siniri: -90 ... +90 derece");
  Serial.println("Varsayilan mod: 900 derece | PD merkezleme: %22 ... %30");
  Serial.print("Varsayilan Kp: ");
  Serial.print(centeringKp, 3);
  Serial.print(" | Kd: ");
  Serial.print(centeringKd, 3);
  Serial.print(" | Slew-rate: %");
  Serial.print(centeringSlewRatePercentPerSecond, 1);
  Serial.println("/s");
  Serial.println("Sanal son durak (son 20 derece): %30 ... %100");

  if (isAs5600Connected()) {
    Serial.println("AS5600 Baglantisi Basarili");
  } else {
    Serial.println("HATA: AS5600 I2C adresinde bulunamadi");
  }

  const uint32_t now = millis();
  lastSensorMs = now;
  lastPrintMs = now;
  updateAngle(now);
  printHelp();
}

void loop() {
  const uint32_t now = millis();

  readSerialCommands(now);

  if (now - lastSensorMs >= kSensorPeriodMs) {
    lastSensorMs = now;
    updateAngle(now);
  }

  updatePositionController(now);
  updateCenteringController(now);
  updateFfbController(now);
  serviceMotor(now);

  if (now - lastPrintMs >= kPrintPeriodMs) {
    lastPrintMs = now;
    printStatus();
  }
}
