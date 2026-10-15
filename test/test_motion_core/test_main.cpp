#include <unity.h>

#include "motion_core.h"
#include "scan_sequence.h"

using namespace turntable;

void test_angle_conversions() {
  TEST_ASSERT_EQUAL_INT64(1600, pulsesFromDegrees(90.0));
  TEST_ASSERT_EQUAL_INT64(6400, pulsesFromDegrees(360.0));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.05625, degreesFromPulses(1));
}

void test_absolute_and_fractional_targets() {
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 350.0, normalizeDegrees(-10.0));
  TEST_ASSERT_EQUAL_INT64(-178, absoluteTargetPulses(0, 0, 350, DirectionPolicy::Shortest));
  TEST_ASSERT_EQUAL_INT64(6222, absoluteTargetPulses(0, 0, 350, DirectionPolicy::Clockwise));
  TEST_ASSERT_EQUAL_INT64(-178, absoluteTargetPulses(0, 0, 350, DirectionPolicy::Counterclockwise));
  long double exact = 0;
  int64_t target = 0;
  for (int i = 0; i < 10; ++i) {
    target = relativeTargetPulses(exact, 0.1);
    exact = relativePositionExactPulses(exact, 0.1);
  }
  TEST_ASSERT_EQUAL_INT64(18, target);
  TEST_ASSERT_DOUBLE_WITHIN(1e-8, 17.7777777778, static_cast<double>(exact));
}

void test_scan_generation() {
  ScanDefinition scan;
  scan.views = 36;
  TEST_ASSERT_EQUAL_UINT16(36, scanViewCount(scan));
  TEST_ASSERT_EQUAL_INT64(100, scanTargetPulses(100, scan, 0));
  TEST_ASSERT_EQUAL_INT64(278, scanTargetPulses(100, scan, 1));
  TEST_ASSERT_EQUAL_INT64(6322, scanTargetPulses(100, scan, 35));
  TEST_ASSERT_EQUAL_INT64(100, scanTargetPulses(100, scan, 36));
  scan.mode = ScanMode::Increment;
  scan.totalDegrees = 100;
  scan.incrementDegrees = 30;
  TEST_ASSERT_EQUAL_UINT16(4, scanViewCount(scan));
  TEST_ASSERT_EQUAL_INT64(1600, scanTargetPulses(0, scan, 3));
  scan.incrementDegrees = 0;
  TEST_ASSERT_FALSE(validScan(scan));
}

void test_scan_state_transitions() {
  ScanDefinition definition;
  definition.views = 36;
  ScanSequence sequence;
  TEST_ASSERT_TRUE(sequence.begin(definition, 0, true, false));
  sequence.beginView(0);
  sequence.reachedTarget();
  TEST_ASSERT_TRUE(sequence.settlingComplete());
  TEST_ASSERT_TRUE(sequence.requestPause());
  TEST_ASSERT_TRUE(sequence.ready);
  TEST_ASSERT_FALSE(sequence.resume());
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ScanSequence::AcknowledgeResult::Advanced),
                        static_cast<int>(sequence.acknowledge()));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ScanSequence::AcknowledgeResult::Invalid),
                        static_cast<int>(sequence.acknowledge()));
  const int64_t target = sequence.targetPulse;
  TEST_ASSERT_TRUE(sequence.requestPause());
  sequence.stoppedAfterPause(false);
  TEST_ASSERT_TRUE(sequence.resume());
  TEST_ASSERT_EQUAL_INT64(target, sequence.targetPulse);
  sequence.reachedTarget();
  TEST_ASSERT_TRUE(sequence.settlingComplete());
  TEST_ASSERT_TRUE(sequence.requestCancel(true));
  sequence.cancelStopped();
  TEST_ASSERT_FALSE(sequence.active);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ScanState::Cancelled),
                        static_cast<int>(sequence.state));
}

void test_settings_validation() {
  TEST_ASSERT_TRUE(validMotionSettings(30, 10, 500, 1200));
  TEST_ASSERT_FALSE(validMotionSettings(31, 10, 500, 1200));
  TEST_ASSERT_FALSE(validMotionSettings(20, 31, 500, 1200));
  TEST_ASSERT_TRUE(validScanDefaults(360, 36));
  TEST_ASSERT_FALSE(validScanDefaults(3601, 36));
  TEST_ASSERT_TRUE(validApCredentials(1, 8));
  TEST_ASSERT_FALSE(validApCredentials(33, 8));
  TEST_ASSERT_FALSE(validApCredentials(20, 7));
  TEST_ASSERT_EQUAL_INT64(0, pulsesFromDegrees(std::numeric_limits<double>::infinity()));
}

void setup() {
  UNITY_BEGIN();
  RUN_TEST(test_angle_conversions);
  RUN_TEST(test_absolute_and_fractional_targets);
  RUN_TEST(test_scan_generation);
  RUN_TEST(test_scan_state_transitions);
  RUN_TEST(test_settings_validation);
  UNITY_END();
}

void loop() {}
