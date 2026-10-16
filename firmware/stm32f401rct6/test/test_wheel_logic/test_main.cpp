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
                                           25.0F};
  CHECK(wheel::computeCenteringDemand(10.0F, 0.0F, params) < 0.0F);
  CHECK(wheel::computeCenteringDemand(-10.0F, 0.0F, params) > 0.0F);
  CHECK(wheel::computeCenteringDemand(0.2F, 0.0F, params) == 0.0F);

  const float positiveLimit =
      wheel::applyVirtualEndstop(30.0F, 450.0F, 900U, 20.0F, 30.0F);
  const float negativeLimit =
      wheel::applyVirtualEndstop(-30.0F, -450.0F, 900U, 20.0F, 30.0F);
  CHECK(positiveLimit == -30.0F);
  CHECK(negativeLimit == 30.0F);
  CHECK(std::fabs(wheel::applyVirtualEndstop(
            100.0F, 449.0F, 900U, 20.0F, 30.0F)) <= 30.0F);

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
  testConfigurationAndHidScaling();
  if (failures == 0) {
    std::puts("PASS: WheelLogic host tests");
  }
  return failures == 0 ? 0 : 1;
}
