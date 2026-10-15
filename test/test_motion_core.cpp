#include <cassert>
#include <cmath>
#include <iostream>

#include "motion_core.h"
#include "scan_sequence.h"

using namespace turntable;

int main() {
  assert(pulsesFromDegrees(90.0) == 1600);
  assert(pulsesFromDegrees(360.0) == 6400);
  assert(std::abs(degreesFromPulses(1) - 0.05625) < 1e-9);

  int64_t zero = 0;
  for (int i = 1; i <= 1000; ++i) {
    const int64_t target = pulsesFromDegrees(i * 0.1);
    assert(target == pulsesFromDegrees(i * 0.1));
    zero = target;
  }
  assert(zero == 1778);
  long double exactPulses = 0;
  int64_t repeatedTarget = 0;
  for (int i = 0; i < 10; ++i) {
    repeatedTarget = relativeTargetPulses(exactPulses, 0.1);
    exactPulses += 0.1L * config::PULSES_PER_TABLE_REV / 360.0L;
  }
  assert(repeatedTarget == 18);

  assert(std::abs(normalizeDegrees(-10.0) - 350.0) < 1e-9);
  assert(absoluteTargetPulses(0, 0, 350, DirectionPolicy::Shortest) == -178);
  assert(absoluteTargetPulses(0, 0, 350, DirectionPolicy::Clockwise) == 6222);
  assert(absoluteTargetPulses(0, 0, 350, DirectionPolicy::Counterclockwise) == -178);

  ScanDefinition scan;
  scan.views = 36;
  assert(scanViewCount(scan) == 36);
  assert(scanTargetPulses(100, scan, 0) == 100);
  assert(scanTargetPulses(100, scan, 1) == 278);
  assert(scanTargetPulses(100, scan, 35) == 6322);
  assert(scanTargetPulses(100, scan, 36) == 100);
  scan.mode = ScanMode::Increment;
  scan.totalDegrees = 100.0;
  scan.incrementDegrees = 30.0;
  assert(scanViewCount(scan) == 4);
  assert(scanTargetPulses(0, scan, 3) == 1600);
  scan.incrementDegrees = 0.0;
  assert(!validScan(scan));

  ScanSequence sequence;
  scan = ScanDefinition{};
  scan.views = 36;
  assert(sequence.begin(scan, 100, true, false));
  sequence.beginView(0);
  sequence.reachedTarget();
  assert(sequence.settlingComplete());
  assert(sequence.ready && sequence.state == ScanState::Ready);
  assert(sequence.requestPause());
  assert(sequence.ready && sequence.state == ScanState::Paused);
  assert(!sequence.resume());
  assert(sequence.state == ScanState::Ready);
  assert(sequence.acknowledge() == ScanSequence::AcknowledgeResult::Advanced);
  const int64_t interruptedTarget = sequence.targetPulse;
  assert(sequence.requestPause());
  sequence.stoppedAfterPause(false);
  assert(sequence.state == ScanState::Paused && !sequence.ready);
  assert(sequence.resume());
  assert(sequence.targetPulse == interruptedTarget);
  sequence.reachedTarget();
  assert(sequence.settlingComplete());
  assert(sequence.acknowledge() == ScanSequence::AcknowledgeResult::Advanced);
  assert(sequence.requestCancel(true));
  sequence.cancelStopped();
  assert(!sequence.active && sequence.state == ScanState::Cancelled);

  assert(sequence.begin(scan, 0, true, false));
  sequence.beginView(0);
  sequence.reachedTarget();
  sequence.settlingComplete();
  for (int i = 0; i < 35; ++i) {
    assert(sequence.acknowledge() == ScanSequence::AcknowledgeResult::Advanced);
    sequence.beginView(sequence.view);
    sequence.reachedTarget();
    sequence.settlingComplete();
  }
  assert(sequence.acknowledge() == ScanSequence::AcknowledgeResult::ViewsComplete);
  sequence.beginReturn();
  assert(sequence.state == ScanState::Returning);
  assert(sequence.requestPause());
  sequence.stoppedAfterPause(false);
  assert(sequence.resume() && sequence.returning);
  sequence.finish(ScanState::Complete);
  assert(!sequence.active && sequence.state == ScanState::Complete);

  assert(validMotionSettings(30, 10, 500, 1200));
  assert(!validMotionSettings(31, 10, 500, 1200));
  assert(!validMotionSettings(20, 31, 500, 1200));
  assert(validScanDefaults(360, 36));
  assert(!validScanDefaults(3601, 36));
  assert(validApCredentials(1, 8));
  assert(!validApCredentials(33, 8));
  assert(!validApCredentials(20, 7));
  assert(pulsesFromDegrees(std::numeric_limits<double>::infinity()) == 0);
  std::cout << "motion core tests passed\n";
}
