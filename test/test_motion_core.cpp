#include <cassert>
#include <cmath>
#include <iostream>

#include "motion_core.h"

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

  assert(validMotionSettings(30, 10, 500, 1200));
  assert(!validMotionSettings(31, 10, 500, 1200));
  assert(!validMotionSettings(20, 31, 500, 1200));
  assert(!pulsesFromDegrees(std::numeric_limits<double>::infinity()));
  std::cout << "motion core tests passed\n";
}
