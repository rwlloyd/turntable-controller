#pragma once

#include <cstdint>

namespace config {
constexpr uint8_t STEP_PIN = 26;
constexpr uint8_t DIR_PIN = 27;
constexpr uint8_t ENABLE_PIN = 25;
constexpr bool ENABLE_ACTIVE_LOW = true;

constexpr int32_t FULL_STEPS_PER_MOTOR_REV = 200;
constexpr int32_t MICROSTEPS = 16;
constexpr int32_t MOTOR_REVS_PER_TABLE_REV = 2;
constexpr int32_t PULSES_PER_TABLE_REV =
    FULL_STEPS_PER_MOTOR_REV * MICROSTEPS * MOTOR_REVS_PER_TABLE_REV;
constexpr double DEGREES_PER_REV = 360.0;

constexpr float DEFAULT_MAX_SPEED_DPS = 30.0f;
constexpr float DEFAULT_ACCEL_DPS2 = 10.0f;
constexpr float MAX_ACCEL_DPS2 = 30.0f;
constexpr uint32_t DEFAULT_SETTLE_MS = 500;
constexpr uint32_t DEFAULT_JOG_LEASE_MS = 1200;
constexpr uint32_t MIN_JOG_LEASE_MS = 300;
constexpr uint32_t MAX_JOG_LEASE_MS = 5000;
constexpr float MIN_SPEED_DPS = 0.5f;
constexpr float MAX_SPEED_DPS = 30.0f;
constexpr float MAX_SCAN_ANGLE = 3600.0f;
constexpr uint16_t MAX_SCAN_VIEWS = 1000;
constexpr uint16_t DEFAULT_SCAN_VIEWS = 36;
constexpr uint32_t STATUS_INTERVAL_MS = 250;

constexpr char DEFAULT_AP_SSID[] = "Turntable-Setup";
constexpr char DEFAULT_AP_PASSWORD[] = "turntable";
constexpr char AP_ADDRESS[] = "192.168.4.1";
}  // namespace config
