#include <WheelLogic.h>

#include <cmath>
#include <cstdint>
#include <cstdio>

namespace {
int failures = 0;

void check(bool condition, const char* expression, int line) {
  if (!condition) {
    std::printf("FAIL line %d: %s\n", line, expression);
    ++failures;
  }
}

#define CHECK(expression) check((expression), #expression, __LINE__)

void testPwmConversionAndMotorDirections() {
  const uint8_t percentages[] = {0U, 1U, 10U, 22U, 30U, 50U, 75U, 100U};
  const uint8_t expected[] = {0U, 3U, 26U, 56U, 77U, 128U, 191U, 255U};
  uint8_t previous = 0U;
  for (size_t index = 0; index < sizeof(percentages); ++index) {
    const uint8_t duty = wheel::pwmPercentToDuty(percentages[index]);
    CHECK(duty == expected[index]);
    CHECK(duty >= previous);
    previous = duty;
  }

  const wheel::MotorDrive right = wheel::makeMotorDrive(
      wheel::MotorDirection::kRight, 50U);
  CHECK(right.enable && right.rightDuty == 128U && right.leftDuty == 0U);
  const wheel::MotorDrive left = wheel::makeMotorDrive(
      wheel::MotorDirection::kLeft, 50U);
  CHECK(left.enable && left.rightDuty == 0U && left.leftDuty == 128U);
  const wheel::MotorDrive stopped = wheel::makeMotorDrive(
      wheel::MotorDirection::kStopped, 100U);
  CHECK(!stopped.enable && stopped.rightDuty == 0U &&
        stopped.leftDuty == 0U);
  const wheel::MotorDrive limited = wheel::makeMotorDrive(
      wheel::MotorDirection::kRight, 100U, 30U);
  CHECK(limited.rightDuty == wheel::pwmPercentToDuty(30U));
}

void testEncoderUnwrapAndReconnect() {
  CHECK(wheel::unwrapDelta(0U, 4095U) == 1);
  CHECK(wheel::unwrapDelta(4095U, 0U) == -1);
  CHECK(wheel::unwrapDelta(101U, 100U) == 1);
  CHECK(wheel::unwrapDelta(100U, 101U) == -1);

  wheel::PositionEstimator estimator;
  estimator.initialize(4090U, 0, true);
  estimator.update(5U, 10000U, 1, 0.025F);
  CHECK(estimator.counts() == 11);
  estimator.update(4090U, 10000U, 1, 0.025F);
  CHECK(estimator.counts() == 0);
  estimator.update(4000U, 10000U, -1, 0.025F);
  CHECK(estimator.counts() == 90);

  estimator.resynchronize(1234U);
  CHECK(!estimator.trusted());
  const int32_t before = estimator.counts();
  estimator.update(1235U, 10000U, 1, 0.025F);
  CHECK(estimator.counts() == before + 1);
  estimator.zero(1235U);
  CHECK(estimator.trusted() && estimator.counts() == 0);
}

void testDeadTimeAndTimeouts() {
  wheel::MotorGate gate;
  wheel::MotorCommand output{};
  CHECK(gate.request(wheel::MotorDirection::kRight, 10U,
                     wheel::MotionOwner::kManual, 100U, 50U, output) ==
        wheel::MotorGateAction::kWaiting);
  CHECK(!gate.active() && gate.pending());
  CHECK(gate.service(149U, output) == wheel::MotorGateAction::kWaiting);
  CHECK(gate.service(150U, output) == wheel::MotorGateAction::kApply);
  CHECK(gate.active() && output.direction == wheel::MotorDirection::kRight);

  CHECK(gate.request(wheel::MotorDirection::kLeft, 10U,
                     wheel::MotionOwner::kManual, 151U, 50U, output) ==
        wheel::MotorGateAction::kWaiting);
  CHECK(!gate.active());
  CHECK(gate.service(200U, output) == wheel::MotorGateAction::kWaiting);
  CHECK(gate.service(201U, output) == wheel::MotorGateAction::kApply);
  CHECK(output.direction == wheel::MotorDirection::kLeft);

  CHECK(!wheel::timeReached(199U, 100U, 100U));
  CHECK(wheel::timeReached(200U, 100U, 100U));
  CHECK(wheel::timeReached(0x00000010UL, 0xFFFFFFF0UL, 0x20UL));
  CHECK(!wheel::timeReached(0x0000000FUL, 0xFFFFFFF0UL, 0x20UL));
}

void testPdEndstopSlewAndDirection() {
  const wheel::CenteringParameters params{0.4F, 0.025F, 0.5F, 3.0F,
                                           30.0F};  // PD max = %30
  CHECK(wheel::computeCenteringDemand(10.0F, 0.0F, params) < 0.0F);
  CHECK(wheel::computeCenteringDemand(-10.0F, 0.0F, params) > 0.0F);
  CHECK(wheel::computeCenteringDemand(0.2F, 0.0F, params) == 0.0F);

  // --- Sanal sınır algoritması (C3 ile birebir) ---
  // 900° profil: limit=450, rampStart=430, startPwm=30, maxPwm=100
  auto endstopMag = [](float angle) {
    return std::fabs(wheel::applyVirtualEndstop(
        0.0F, angle, 900U, 20.0F, 30.0F, 100.0F));
  };
  CHECK(std::fabs(endstopMag(430.0F) - 30.0F) < 0.01F);
  CHECK(std::fabs(endstopMag(435.0F) - 40.94F) < 0.01F);
  CHECK(std::fabs(endstopMag(440.0F) - 65.0F) < 0.01F);
  CHECK(std::fabs(endstopMag(445.0F) - 89.06F) < 0.01F);
  CHECK(std::fabs(endstopMag(450.0F) - 100.0F) < 0.01F);

  // Endstop kuvveti merkeze doğru: pozitif açı -> negatif PWM
  CHECK(wheel::applyVirtualEndstop(0.0F, 440.0F, 900U, 20.0F, 30.0F,
                                   100.0F) < 0.0F);
  CHECK(wheel::applyVirtualEndstop(0.0F, -440.0F, 900U, 20.0F, 30.0F,
                                   100.0F) > 0.0F);

  // 360° profil: son 20° aralığı ±160 ile ±180
  auto endstopMag360 = [](float angle) {
    return std::fabs(wheel::applyVirtualEndstop(
        0.0F, angle, 360U, 20.0F, 30.0F, 100.0F));
  };
  CHECK(std::fabs(endstopMag360(170.0F) - 65.0F) < 0.01F);
  CHECK(std::fabs(endstopMag360(180.0F) - 100.0F) < 0.01F);
  CHECK(endstopMag360(159.0F) < 0.01F);

  // 1080° profil: son 20° aralığı ±520 ile ±540
  auto endstopMag1080 = [](float angle) {
    return std::fabs(wheel::applyVirtualEndstop(
        0.0F, angle, 1080U, 20.0F, 30.0F, 100.0F));
  };
  CHECK(std::fabs(endstopMag1080(530.0F) - 65.0F) < 0.01F);
  CHECK(std::fabs(endstopMag1080(540.0F) - 100.0F) < 0.01F);

  // Endstop dışında istenen değer aynen döner (endstop kesmez)
  CHECK(wheel::applyVirtualEndstop(20.0F, 100.0F, 900U, 20.0F, 30.0F,
                                   100.0F) == 20.0F);

  // Endstop dışında istenen değer aynen döner (endstop kesmez)
  CHECK(wheel::applyVirtualEndstop(20.0F, 100.0F, 900U, 20.0F, 30.0F,
                                   100.0F) == 20.0F);

  CHECK(wheel::directionForDemand(1.0F, 1) ==
        wheel::MotorDirection::kRight);
  CHECK(wheel::directionForDemand(-1.0F, 1) ==
        wheel::MotorDirection::kLeft);
  CHECK(wheel::directionForDemand(1.0F, -1) ==
        wheel::MotorDirection::kLeft);

  wheel::SlewLimiter limiter;
  CHECK(std::fabs(limiter.update(100.0F, 80.0F, 100000U) - 8.0F) <
        0.001F);
  CHECK(std::fabs(limiter.update(-100.0F, 80.0F, 100000U)) < 0.001F);
}

void testCenteringPwmFloor() {
  // Merkezleme kabul matrisi: applied -> nihai PWM.
  // 0'dan kucukler 0; 0.5'den buyukler %25'e sabitlenir; 25+ ayni kalir.
  CHECK(wheel::centeringAppliedOutputToPwm(0.0F) == 0U);
  CHECK(wheel::centeringAppliedOutputToPwm(0.3F) == 0U);
  CHECK(wheel::centeringAppliedOutputToPwm(0.49F) == 0U);

  CHECK(wheel::centeringAppliedOutputToPwm(0.5F) == 25U);
  CHECK(wheel::centeringAppliedOutputToPwm(1.0F) == 25U);
  CHECK(wheel::centeringAppliedOutputToPwm(5.0F) == 25U);
  CHECK(wheel::centeringAppliedOutputToPwm(10.0F) == 25U);
  CHECK(wheel::centeringAppliedOutputToPwm(15.0F) == 25U);
  CHECK(wheel::centeringAppliedOutputToPwm(22.0F) == 25U);
  CHECK(wheel::centeringAppliedOutputToPwm(24.0F) == 25U);
  CHECK(wheel::centeringAppliedOutputToPwm(24.4F) == 25U);

  CHECK(wheel::centeringAppliedOutputToPwm(25.0F) == 25U);
  CHECK(wheel::centeringAppliedOutputToPwm(30.0F) == 30U);
  CHECK(wheel::centeringAppliedOutputToPwm(35.0F) == 35U);

  // Negatif degerler: magnitude ayni, isaret cagiran tarafinda korunur.
  CHECK(wheel::centeringAppliedOutputToPwm(-1.0F) == 25U);
  CHECK(wheel::centeringAppliedOutputToPwm(-8.0F) == 25U);
  CHECK(wheel::centeringAppliedOutputToPwm(-25.0F) == 25U);
  CHECK(wheel::centeringAppliedOutputToPwm(-26.0F) == 26U);
  CHECK(wheel::centeringAppliedOutputToPwm(-35.0F) == 35U);

  // %1-24 fiziksel PWM asla uretilememeli.
  for (float applied = 0.5F; applied < 25.0F; applied += 0.5F) {
    CHECK(wheel::centeringAppliedOutputToPwm(applied) == 25U);
  }
}

void testSlewNeverYields1to24Percent() {
  wheel::SlewLimiter limiter;
  // 500 Hz dongu, slew = 200 %/s => adim basina ~0.4 %.
  // Sifirdan 35'e dogru cikarken fisiksel PWM 0 veya >=25 olmali.
  bool saw25 = false;
  uint8_t lastPwm = 0U;
  for (int step = 0; step < 400; ++step) {
    const float applied = limiter.update(35.0F, 200.0F, 2000U);
    const uint8_t pwm = wheel::centeringAppliedOutputToPwm(applied);
    CHECK(pwm == 0U || pwm >= 25U);
    if (pwm == 25U) {
      saw25 = true;
    }
    lastPwm = pwm;
  }
  CHECK(saw25);
  CHECK(lastPwm == 35U);

  // Slew hizli yükselme testi: 0 -> 35 talebi 200 %/s ile ~175 ms'de ulasmali.
  wheel::SlewLimiter limiter2;
  bool reached35 = false;
  for (int step = 0; step < 1000; ++step) {
    const float applied = limiter2.update(35.0F, 200.0F, 2000U);
    if (applied >= 34.9F) {
      reached35 = true;
      break;
    }
  }
  CHECK(reached35);
}

void testPwmLimitsPerMode() {
  wheel::WheelSettings settings = wheel::defaultSettings();

  // Manuel PWM max: %50
  CHECK(settings.maximumPwmPercent == 50U);
  // PD merkezleme max: %35
  CHECK(settings.maximumCenteringPwmPercent == 35U);
  // Sanal sınır max: %100 (PD'den bağımsız)
  CHECK(settings.maximumEndstopPwmPercent == 100U);
  // GAME_FFB max: %90
  CHECK(settings.gameFfbMaxPwmPercent == 90U);
  // FFB kazanci: %100
  CHECK(settings.ffbGainPercent == 100U);

  // settingsAreValid: endstop %100'e kadar olabilir, PD %30'a kadar
  wheel::WheelSettings valid = settings;
  valid.maximumPwmPercent = 50U;
  valid.maximumCenteringPwmPercent = 30U;
  valid.maximumEndstopPwmPercent = 100U;
  valid.gameFfbMaxPwmPercent = 90U;
  CHECK(wheel::settingsAreValid(valid));

  // Endstop > 50 olabilir (PD limitinden bağımsız)
  wheel::WheelSettings endstop100 = settings;
  endstop100.maximumEndstopPwmPercent = 100U;
  endstop100.maximumPwmPercent = 50U;
  endstop100.maximumCenteringPwmPercent = 30U;
  CHECK(wheel::settingsAreValid(endstop100));

  // GAME_FFB max 90'dan fazla olamaz
  wheel::WheelSettings invalidGf = settings;
  invalidGf.gameFfbMaxPwmPercent = 91U;
  CHECK(!wheel::settingsAreValid(invalidGf));

  // Donanım PWM dönüşümü %100 destekler
  CHECK(wheel::pwmPercentToDuty(100U) == 255U);
  CHECK(wheel::pwmPercentToDuty(50U) == 128U);
}

void testGameFfbLimits() {
  // Varsayilan ayarlari kontrol et
  wheel::WheelSettings settings = wheel::defaultSettings();
  CHECK(settings.gameFfbMaxPwmPercent == 90U);
  CHECK(settings.ffbGainPercent == 100U);
  CHECK(settings.ffbDeadzonePercent == 5U);
  CHECK(wheel::settingsAreValid(settings));

  // ffbForceToPwm: sifir kuvvet -> 0 PWM
  CHECK(wheel::ffbForceToPwm(0, 100, 5, 90) == 0U);

  // Deadzone'nin altinda 0 PWM (kucuk gucleri %22'ye sabitleme yapma)
  CHECK(wheel::ffbForceToPwm(3, 100, 5, 90) == 0U);
  CHECK(wheel::ffbForceToPwm(5, 100, 5, 90) == 0U);

  // Yakin yukarida %22'nin altinda kalmali, %22'ye zorla sabitlenmemeli
  const uint8_t pwm6 = wheel::ffbForceToPwm(6, 100, 5, 90);
  CHECK(pwm6 < 22U);

  // Gain ile faktor: %50 gain, %100 kuvvet -> %50 max'dan yarisi
  CHECK(wheel::ffbForceToPwm(100, 50, 5, 90) ==
        wheel::ffbForceToPwm(50, 100, 5, 90));

  // Max clamp: %100 kuvvet, max %90 -> 90
  CHECK(wheel::ffbForceToPwm(100, 100, 5, 90) == 90U);

  // Max clamp: %100 kuvvet, max %50 -> 50
  CHECK(wheel::ffbForceToPwm(100, 100, 5, 50) == 50U);

  // Negatif kuvvet ayni magnitude
  CHECK(wheel::ffbForceToPwm(-100, 100, 5, 90) == 90U);

  // settingsAreValid: yeni alanlar icin aralik kontrolu
  wheel::WheelSettings invalid = settings;
  invalid.gameFfbMaxPwmPercent = 91U;
  CHECK(!wheel::settingsAreValid(invalid));
  invalid = settings;
  invalid.gameFfbMaxPwmPercent = 0U;
  CHECK(!wheel::settingsAreValid(invalid));
  invalid = settings;
  invalid.ffbGainPercent = 101U;
  CHECK(!wheel::settingsAreValid(invalid));
  invalid = settings;
  invalid.ffbGainPercent = 0U;
  CHECK(!wheel::settingsAreValid(invalid));
  invalid = settings;
  invalid.ffbDeadzonePercent = 51U;
  CHECK(!wheel::settingsAreValid(invalid));

  // CRC kaydi gecerli olmali
  const wheel::SettingsRecord record = wheel::makeSettingsRecord(settings);
  CHECK(wheel::settingsRecordIsValid(record));
}

void testConfigurationAndHidScaling() {
  wheel::WheelSettings settings = wheel::defaultSettings();
  CHECK(wheel::settingsAreValid(settings));
  const wheel::SettingsRecord record = wheel::makeSettingsRecord(settings);
  CHECK(wheel::settingsRecordIsValid(record));

  wheel::SettingsRecord corrupt = record;
  corrupt.settings.centeringKp = 99.0F;
  CHECK(!wheel::settingsRecordIsValid(corrupt));
  corrupt = record;
  corrupt.crc32 ^= 1U;
  CHECK(!wheel::settingsRecordIsValid(corrupt));

  CHECK(wheel::angleToHidAxis(0.0F, 900U, true) == 0);
  CHECK(wheel::angleToHidAxis(450.0F, 900U, true) == 32767);
  CHECK(wheel::angleToHidAxis(-450.0F, 900U, true) == -32767);
  CHECK(wheel::angleToHidAxis(999.0F, 900U, true) == 32767);
  CHECK(wheel::angleToHidAxis(100.0F, 900U, false) == 0);

  wheel::WheelInputState inputs{};
  CHECK(!inputs.pedalsAvailable && inputs.throttle == 0U &&
        inputs.buttons == 0U);
}
}  // namespace

int main() {
  testPwmConversionAndMotorDirections();
  testEncoderUnwrapAndReconnect();
  testDeadTimeAndTimeouts();
  testPdEndstopSlewAndDirection();
  testCenteringPwmFloor();
  testSlewNeverYields1to24Percent();
  testPwmLimitsPerMode();
  testGameFfbLimits();
  testConfigurationAndHidScaling();
  if (failures == 0) {
    std::puts("PASS: WheelLogic host tests");
  }
  return failures == 0 ? 0 : 1;
}
