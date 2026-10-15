#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

#include "config.h"

namespace turntable {

inline bool finite(double value) { return std::isfinite(value); }

inline int64_t pulsesFromDegrees(double degrees) {
  if (!finite(degrees) ||
      std::abs(degrees) > static_cast<double>(std::numeric_limits<int64_t>::max()) /
                              config::PULSES_PER_TABLE_REV * 360.0) {
    return 0;
  }
  return static_cast<int64_t>(std::llround(
      degrees * config::PULSES_PER_TABLE_REV / config::DEGREES_PER_REV));
}

inline double degreesFromPulses(int64_t pulses) {
  return static_cast<double>(pulses) * config::DEGREES_PER_REV /
         config::PULSES_PER_TABLE_REV;
}

inline double normalizeDegrees(double degrees) {
  if (!finite(degrees)) return 0.0;
  double result = std::fmod(degrees, config::DEGREES_PER_REV);
  if (result < 0.0) result += config::DEGREES_PER_REV;
  return result;
}

enum class DirectionPolicy { Shortest, Clockwise, Counterclockwise };

inline int64_t absoluteTargetPulses(int64_t current, int64_t zero,
                                    double angleDegrees,
                                    DirectionPolicy policy) {
  const int64_t modulo = config::PULSES_PER_TABLE_REV;
  const int64_t requested = pulsesFromDegrees(normalizeDegrees(angleDegrees));
  const int64_t currentRelative = current - zero;
  const int64_t remainder = ((requested - currentRelative) % modulo + modulo) % modulo;
  int64_t delta = remainder;
  if (policy == DirectionPolicy::Counterclockwise) {
    if (delta != 0) delta -= modulo;
  } else if (policy == DirectionPolicy::Shortest) {
    if (delta > modulo / 2) delta -= modulo;
  }
  return current + delta;
}

enum class ScanMode { ViewCount, Increment };

struct ScanDefinition {
  double totalDegrees = 360.0;
  ScanMode mode = ScanMode::ViewCount;
  uint16_t views = config::DEFAULT_SCAN_VIEWS;
  double incrementDegrees = 10.0;
};

inline bool validScan(const ScanDefinition& scan) {
  if (!finite(scan.totalDegrees) || scan.totalDegrees <= 0.0 ||
      scan.totalDegrees > config::MAX_SCAN_ANGLE) {
    return false;
  }
  if (scan.mode == ScanMode::ViewCount) {
    return scan.views >= 1 && scan.views <= config::MAX_SCAN_VIEWS;
  }
  return finite(scan.incrementDegrees) && scan.incrementDegrees > 0.0 &&
         scan.incrementDegrees <= scan.totalDegrees;
}

inline uint16_t scanViewCount(const ScanDefinition& scan) {
  if (!validScan(scan)) return 0;
  if (scan.mode == ScanMode::ViewCount) return scan.views;
  const double count = std::ceil(scan.totalDegrees / scan.incrementDegrees);
  return static_cast<uint16_t>(count);
}

inline int64_t scanTargetPulses(int64_t start, const ScanDefinition& scan,
                                uint16_t index) {
  const uint16_t count = scanViewCount(scan);
  if (index >= count) return start;
  double offsetDegrees = 0.0;
  if (scan.mode == ScanMode::ViewCount) {
    offsetDegrees = scan.totalDegrees * static_cast<double>(index) / scan.views;
  } else {
    offsetDegrees = scan.incrementDegrees * index;
  }
  return start + pulsesFromDegrees(offsetDegrees);
}

inline bool validMotionSettings(float maxSpeed, float acceleration,
                                uint32_t settleMs, uint32_t jogLeaseMs) {
  return std::isfinite(maxSpeed) && maxSpeed >= config::MIN_SPEED_DPS &&
         maxSpeed <= config::MAX_SPEED_DPS && std::isfinite(acceleration) &&
         acceleration >= config::MIN_SPEED_DPS &&
         acceleration <= config::MAX_ACCEL_DPS2 && settleMs <= 60000 &&
         jogLeaseMs >= config::MIN_JOG_LEASE_MS &&
         jogLeaseMs <= config::MAX_JOG_LEASE_MS;
}

}  // namespace turntable
