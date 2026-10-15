#include <Arduino.h>
#include <ArduinoJson.h>
#include <AccelStepper.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <algorithm>
#include <climits>
#include <cstring>
#include <cstdlib>
#include <vector>

#include "config.h"
#include "motion_core.h"
#include "scan_sequence.h"

namespace {
using turntable::DirectionPolicy;
using turntable::ScanDefinition;
using turntable::ScanMode;
using turntable::ScanSequence;
using turntable::ScanState;

enum class MotionKind { Idle, Move, Jog, Continuous, Scan, Return };

struct Settings {
  String ssid = config::DEFAULT_AP_SSID;
  String password = config::DEFAULT_AP_PASSWORD;
  float maxSpeedDps = config::DEFAULT_MAX_SPEED_DPS;
  float accelerationDps2 = config::DEFAULT_ACCEL_DPS2;
  uint32_t settleMs = config::DEFAULT_SETTLE_MS;
  uint32_t jogLeaseMs = config::DEFAULT_JOG_LEASE_MS;
  float defaultScanDegrees = 360.0f;
  uint16_t defaultScanViews = config::DEFAULT_SCAN_VIEWS;
  bool autoAdvanceDefault = false;
  bool returnToStartDefault = false;
};

AccelStepper stepper(AccelStepper::DRIVER, config::STEP_PIN, config::DIR_PIN);
AsyncWebServer server(80);
AsyncWebSocket websocket("/api/v1/events");
Preferences preferences;
SemaphoreHandle_t controllerMutex = nullptr;
Settings settings;

MotionKind motion = MotionKind::Idle;
bool motorEnabled = false;
bool positionVerified = false;
ScanSequence scanSequence;
ScanState& scanState = scanSequence.state;
ScanDefinition& scan = scanSequence.definition;
bool& scanActive = scanSequence.active;
bool& scanPaused = scanSequence.paused;
bool& scanPausePending = scanSequence.pausePending;
bool& scanCancelPending = scanSequence.cancelPending;
bool& returnToStart = scanSequence.returnToStart;
bool& autoAdvance = scanSequence.autoAdvance;
bool& readyView = scanSequence.ready;
bool pendingDisable = false;
bool manualStopPending = false;
int jogDirection = 0;
int continuousDirection = 0;
uint16_t& scanView = scanSequence.view;
uint16_t& scanViews = scanSequence.viewCount;
float activeSpeedDps = config::DEFAULT_MAX_SPEED_DPS;
int64_t zeroPulse = 0;
long double exactPositionPulses = 0.0L;
long expectedTargetPulse = 0;
int64_t& scanStartPulse = scanSequence.startPulse;
int64_t& scanTargetPulse = scanSequence.targetPulse;
uint32_t jogExpiry = 0;
uint32_t settleDeadline = 0;
uint32_t lastStatus = 0;
String scanMessage;
std::vector<String> operationIds;
char bodyHandledMarker;

void lockController() { xSemaphoreTake(controllerMutex, portMAX_DELAY); }
void unlockController() { xSemaphoreGive(controllerMutex); }

const char* motionName() {
  switch (motion) {
    case MotionKind::Move: return "moving";
    case MotionKind::Jog: return "jogging";
    case MotionKind::Continuous: return "continuous";
    case MotionKind::Scan: return "scan";
    case MotionKind::Return: return "returning";
    default: return "idle";
  }
}

const char* scanName() {
  switch (scanState) {
    case ScanState::Moving: return "moving";
    case ScanState::Settling: return "settling";
    case ScanState::Ready: return "ready";
    case ScanState::Pausing: return "pausing";
    case ScanState::Paused: return "paused";
    case ScanState::Returning: return "returning";
    case ScanState::Canceling: return "canceling";
    case ScanState::Complete: return "complete";
    case ScanState::Cancelled: return "cancelled";
    case ScanState::Failed: return "failed";
    default: return "idle";
  }
}

int64_t currentPulses() { return stepper.currentPosition(); }
double estimatedAngle() {
  return turntable::normalizeDegrees(
      turntable::degreesFromPulses(currentPulses() - zeroPulse));
}

void setEnabled(bool enabled) {
  if (enabled == motorEnabled) return;
  if (enabled) {
    stepper.enableOutputs();
    motorEnabled = true;
  } else {
    stepper.disableOutputs();
    motorEnabled = false;
    positionVerified = false;
  }
}

void setMotionLimits(float speedDps) {
  activeSpeedDps = std::min(speedDps, settings.maxSpeedDps);
  stepper.setMaxSpeed(activeSpeedDps * config::PULSES_PER_TABLE_REV / 360.0f);
  stepper.setAcceleration(settings.accelerationDps2 *
                          config::PULSES_PER_TABLE_REV / 360.0f);
}

void moveToPulses(int64_t target, MotionKind kind, float speedDps) {
  setEnabled(true);
  setMotionLimits(speedDps);
  motion = kind;
  expectedTargetPulse = static_cast<long>(target);
  stepper.moveTo(static_cast<long>(target));
}

bool isIdle() { return motion == MotionKind::Idle && stepper.distanceToGo() == 0; }

void startStop() {
  if (motion == MotionKind::Idle) return;
  if (motion == MotionKind::Scan) {
    scanPausePending = true;
  } else if (motion == MotionKind::Jog || motion == MotionKind::Continuous) {
    manualStopPending = true;
  }
  stepper.stop();
}

void clearScan(ScanState finalState, const String& message) {
  scanSequence.finish(finalState);
  scanMessage = message;
  if (motion == MotionKind::Scan) motion = MotionKind::Idle;
}

void beginScanView(uint16_t index) {
  scanSequence.beginView(index);
  motion = MotionKind::Scan;
  if (scanTargetPulse == currentPulses()) {
    scanSequence.reachedTarget();
    settleDeadline = millis() + settings.settleMs;
  } else {
    moveToPulses(scanTargetPulse, MotionKind::Scan, activeSpeedDps);
  }
}

void finishScan() {
  if (returnToStart && positionVerified) {
    scanSequence.beginReturn();
    if (scanTargetPulse == currentPulses()) {
      clearScan(ScanState::Complete, "Scan complete; already at starting position.");
    } else {
      moveToPulses(scanTargetPulse, MotionKind::Return, activeSpeedDps);
    }
  } else {
    clearScan(ScanState::Complete,
              returnToStart ? "Scan complete; return skipped because position is unverified."
                            : "Scan complete.");
  }
}

void advanceScan() {
  if (!scanActive || scanState != ScanState::Ready) return;
  const auto result = scanSequence.acknowledge();
  if (result == ScanSequence::AcknowledgeResult::Invalid) return;
  if (result == ScanSequence::AcknowledgeResult::ViewsComplete) {
    finishScan();
    return;
  }
  beginScanView(scanView);
}

JsonDocument makeStatus() {
  JsonDocument doc;
  doc["ok"] = true;
  doc["position"]["estimatedAngle"] = estimatedAngle();
  doc["position"]["unwrappedDegrees"] =
      turntable::degreesFromPulses(currentPulses() - zeroPulse);
  doc["position"]["absolutePulses"] = currentPulses();
  doc["position"]["zeroPulse"] = zeroPulse;
  doc["position"]["confidence"] = positionVerified ? "estimated" : "unverified";
  doc["position"]["verified"] = positionVerified;
  doc["motor"]["enabled"] = motorEnabled;
  doc["motion"] = motionName();
  doc["speedDegreesPerSecond"] = activeSpeedDps;
  doc["scan"]["active"] = scanActive;
  doc["scan"]["state"] = scanName();
  doc["scan"]["viewIndex"] = scanActive ? scanView + 1 : scanView;
  doc["scan"]["viewCount"] = scanViews;
  doc["scan"]["progressPercent"] =
      scanActive && scanViews > 0
          ? static_cast<double>(scanView) * 100.0 / scanViews
          : (scanState == ScanState::Complete ? 100.0 : 0.0);
  doc["scan"]["targetAngle"] =
      turntable::normalizeDegrees(turntable::degreesFromPulses(scanTargetPulse - zeroPulse));
  doc["scan"]["ready"] = readyView;
  doc["scan"]["autoAdvance"] = autoAdvance;
  doc["scan"]["message"] = scanMessage;
  doc["wifi"]["connectedClients"] = WiFi.softAPgetStationNum();
  return doc;
}

String jsonString(JsonDocument& doc) {
  String body;
  serializeJson(doc, body);
  return body;
}

void sendError(AsyncWebServerRequest* request, int status, const char* code,
               const char* message) {
  JsonDocument doc;
  doc["ok"] = false;
  doc["error"]["code"] = code;
  doc["error"]["message"] = message;
  request->send(status, "application/json", jsonString(doc));
}

void sendStatus(AsyncWebServerRequest* request, int status = 200) {
  JsonDocument doc = makeStatus();
  request->send(status, "application/json", jsonString(doc));
}

void broadcastStatus() {
  lockController();
  JsonDocument doc = makeStatus();
  String body = jsonString(doc);
  unlockController();
  websocket.textAll(body);
}

bool checkOperationId(JsonDocument& doc, AsyncWebServerRequest* request,
                      String& operationId) {
  if (!doc["operationId"].is<const char*>()) {
    sendError(request, 400, "operation_id_required",
              "State-changing requests require a non-empty operationId of at most 64 characters.");
    return false;
  }
  operationId = doc["operationId"].as<String>();
  if (operationId.length() == 0 || operationId.length() > 64) {
    sendError(request, 400, "operation_id_required",
              "State-changing requests require a non-empty operationId of at most 64 characters.");
    return false;
  }
  for (const String& known : operationIds) {
    if (known == operationId) {
      JsonDocument duplicate;
      duplicate["ok"] = true;
      duplicate["duplicate"] = true;
      request->send(200, "application/json", jsonString(duplicate));
      return false;
    }
  }
  return true;
}

void rememberOperation(const String& operationId) {
  operationIds.push_back(operationId);
  if (operationIds.size() > 64) operationIds.erase(operationIds.begin());
}

bool bodyNumber(JsonDocument& doc, const char* key, double& value) {
  if (!doc[key].is<double>() && !doc[key].is<int>() && !doc[key].is<long>()) return false;
  value = doc[key].as<double>();
  return turntable::finite(value);
}

bool optionalNumber(JsonDocument& doc, const char* key, double fallback,
                    double& value) {
  if (!doc.containsKey(key)) {
    value = fallback;
    return true;
  }
  return bodyNumber(doc, key, value);
}

bool parseDirection(JsonDocument& doc, int& direction) {
  if (!doc["direction"].is<const char*>()) return false;
  const char* value = doc["direction"].as<const char*>();
  if (value == nullptr) return false;
  if (strcmp(value, "clockwise") == 0) direction = 1;
  else if (strcmp(value, "counterclockwise") == 0) direction = -1;
  else return false;
  return true;
}

bool parsePolicy(JsonDocument& doc, DirectionPolicy& policy) {
  if (doc.containsKey("directionPolicy") &&
      !doc["directionPolicy"].is<const char*>()) return false;
  const char* value = doc["directionPolicy"] | "shortest";
  if (strcmp(value, "shortest") == 0) policy = DirectionPolicy::Shortest;
  else if (strcmp(value, "clockwise") == 0) policy = DirectionPolicy::Clockwise;
  else if (strcmp(value, "counterclockwise") == 0) policy = DirectionPolicy::Counterclockwise;
  else return false;
  return true;
}

void dispatchCommand(AsyncWebServerRequest* request, JsonDocument& doc) {
  const String path = request->url();
  if (path == "/api/v1/settings") {
    double speed, acceleration;
    uint32_t settle = doc["settleMs"] | settings.settleMs;
    uint32_t lease = doc["jogLeaseMs"] | settings.jogLeaseMs;
    float nextSpeed = settings.maxSpeedDps;
    float nextAcceleration = settings.accelerationDps2;
    float scanDegrees = settings.defaultScanDegrees;
    uint16_t scanViewsDefault = settings.defaultScanViews;
    bool autoAdvanceDefault = settings.autoAdvanceDefault;
    bool returnToStartDefault = settings.returnToStartDefault;
    if (doc.containsKey("maxSpeedDegreesPerSecond")) {
      if (!bodyNumber(doc, "maxSpeedDegreesPerSecond", speed)) {
        sendError(request, 400, "invalid_settings", "Maximum speed must be a finite number.");
        return;
      }
      nextSpeed = speed;
    }
    if (doc.containsKey("accelerationDegreesPerSecondSquared")) {
      if (!bodyNumber(doc, "accelerationDegreesPerSecondSquared", acceleration)) {
        sendError(request, 400, "invalid_settings", "Acceleration must be a finite number.");
        return;
      }
      nextAcceleration = acceleration;
    }
    if (doc.containsKey("defaultScanDegrees")) {
      double scanAngle;
      if (!bodyNumber(doc, "defaultScanDegrees", scanAngle)) {
        sendError(request, 400, "invalid_settings", "Default scan angle must be finite.");
        return;
      }
      scanDegrees = scanAngle;
    }
    if (doc.containsKey("defaultScanViews")) {
      if (!doc["defaultScanViews"].is<uint16_t>()) {
        sendError(request, 400, "invalid_settings", "Default scan views must be an integer.");
        return;
      }
      scanViewsDefault = doc["defaultScanViews"].as<uint16_t>();
    }
    if ((doc.containsKey("autoAdvanceDefault") && !doc["autoAdvanceDefault"].is<bool>()) ||
        (doc.containsKey("returnToStartDefault") &&
         !doc["returnToStartDefault"].is<bool>())) {
      sendError(request, 400, "invalid_settings", "Scan behavior defaults must be booleans.");
      return;
    }
    autoAdvanceDefault = doc["autoAdvanceDefault"] | autoAdvanceDefault;
    returnToStartDefault = doc["returnToStartDefault"] | returnToStartDefault;
    if (doc.containsKey("settleMs") && !doc["settleMs"].is<uint32_t>()) {
      sendError(request, 400, "invalid_settings", "settleMs must be an unsigned integer.");
      return;
    }
    if (doc.containsKey("jogLeaseMs") && !doc["jogLeaseMs"].is<uint32_t>()) {
      sendError(request, 400, "invalid_settings", "jogLeaseMs must be an unsigned integer.");
      return;
    }
    if (!turntable::validMotionSettings(nextSpeed, nextAcceleration, settle, lease) ||
        !turntable::validScanDefaults(scanDegrees, scanViewsDefault)) {
      sendError(request, 400, "invalid_settings", "Settings exceed the documented safe software limits.");
      return;
    }
    if ((doc.containsKey("ssid") && !doc["ssid"].is<const char*>()) ||
        (doc.containsKey("password") && !doc["password"].is<const char*>())) {
      sendError(request, 400, "invalid_settings", "SSID and password must be strings.");
      return;
    }
    String ssid = doc["ssid"] | settings.ssid;
    String password = doc["password"] | settings.password;
    if (!turntable::validApCredentials(ssid.length(), password.length())) {
      sendError(request, 400, "invalid_settings", "SSID must be 1-32 characters and password 8-63 characters.");
      return;
    }
    if (!isIdle() || scanActive) {
      sendError(request, 409, "motion_active", "Settings cannot be changed while motion is active.");
      return;
    }
    const bool credentialsChanged = ssid != settings.ssid || password != settings.password;
    bool persisted = true;
    persisted &= preferences.putString("ssid", ssid) == ssid.length();
    persisted &= preferences.putString("password", password) == password.length();
    persisted &= preferences.putFloat("maxSpeed", nextSpeed) > 0;
    persisted &= preferences.putFloat("accel", nextAcceleration) > 0;
    persisted &= preferences.putUInt("settle", settle) > 0;
    persisted &= preferences.putUInt("jogLease", lease) > 0;
    persisted &= preferences.putFloat("scanDegrees", scanDegrees) > 0;
    persisted &= preferences.putUShort("scanViews", scanViewsDefault) > 0;
    persisted &= preferences.putBool("autoAdvance", autoAdvanceDefault) > 0;
    persisted &= preferences.putBool("returnStart", returnToStartDefault) > 0;
    if (!persisted) {
      sendError(request, 500, "settings_persist_failed",
                "NVS could not save all settings; verify settings after reboot.");
      return;
    }
    settings.ssid = ssid;
    settings.password = password;
    settings.maxSpeedDps = nextSpeed;
    settings.accelerationDps2 = nextAcceleration;
    settings.settleMs = settle;
    settings.jogLeaseMs = lease;
    settings.defaultScanDegrees = scanDegrees;
    settings.defaultScanViews = scanViewsDefault;
    settings.autoAdvanceDefault = autoAdvanceDefault;
    settings.returnToStartDefault = returnToStartDefault;
    JsonDocument response;
    response["ok"] = true;
    response["restartRequired"] = credentialsChanged;
    response["message"] = "Settings saved to NVS.";
    request->send(200, "application/json", jsonString(response));
    return;
  }

  String operationId;
  if (!checkOperationId(doc, request, operationId)) return;

  if (path == "/api/v1/move") {
    if (!positionVerified) {
      sendError(request, 409, "position_unverified", "Set software zero before issuing angle-based moves.");
      return;
    }
    if (!isIdle() || scanActive) {
      sendError(request, 409, "motion_conflict", "Another motion or scan is active.");
      return;
    }
    const char* modeNameValue = doc["mode"].is<const char*>()
                                    ? doc["mode"].as<const char*>()
                                    : nullptr;
    double target;
    DirectionPolicy policy;
    if (modeNameValue == nullptr || !bodyNumber(doc, "degrees", target) ||
        std::abs(target) > 3600.0 || !parsePolicy(doc, policy)) {
      sendError(request, 400, "invalid_move", "Provide mode, finite degrees (absolute value <= 3600), and a valid directionPolicy.");
      return;
    }
    double speed;
    if (!optionalNumber(doc, "speedDegreesPerSecond", settings.maxSpeedDps, speed) ||
        !turntable::finite(speed) || speed < config::MIN_SPEED_DPS ||
        speed > settings.maxSpeedDps) {
      sendError(request, 400, "invalid_speed", "Requested speed is outside configured limits.");
      return;
    }
    int64_t targetPulse;
    long double nextExactPosition;
    if (strcmp(modeNameValue, "relative") == 0) {
      targetPulse = turntable::relativeTargetPulses(exactPositionPulses, target);
      nextExactPosition =
          turntable::relativePositionExactPulses(exactPositionPulses, target);
    } else if (strcmp(modeNameValue, "absolute") == 0) {
      targetPulse = turntable::absoluteTargetPulses(currentPulses(), zeroPulse, target, policy);
      nextExactPosition = targetPulse;
    } else {
      sendError(request, 400, "invalid_move", "mode must be relative or absolute.");
      return;
    }
    if (targetPulse > LONG_MAX || targetPulse < LONG_MIN) {
      sendError(request, 400, "invalid_move", "Target exceeds the supported position range.");
      return;
    }
    rememberOperation(operationId);
    exactPositionPulses = nextExactPosition;
    moveToPulses(targetPulse, MotionKind::Move, speed);
    sendStatus(request, 202);
    return;
  }

  if (path == "/api/v1/jog/start") {
    int direction;
    double speed;
    if (!parseDirection(doc, direction) ||
        !optionalNumber(doc, "speedDegreesPerSecond", settings.maxSpeedDps, speed) ||
        !turntable::finite(speed) ||
        speed < config::MIN_SPEED_DPS || speed > settings.maxSpeedDps) {
      sendError(request, 400, "invalid_jog", "Provide clockwise/counterclockwise direction and a valid speed.");
      return;
    }
    if (scanActive || (motion != MotionKind::Idle && motion != MotionKind::Jog)) {
      sendError(request, 409, "motion_conflict", "Jog conflicts with the active operation.");
      return;
    }
    if (motion == MotionKind::Jog &&
        (direction != jogDirection || manualStopPending)) {
      sendError(request, 409, "motion_conflict", "Stop the current jog before changing direction.");
      return;
    }
    if (!positionVerified) {
      sendError(request, 409, "position_unverified", "Set software zero before moving.");
      return;
    }
    if (motion != MotionKind::Jog) {
      manualStopPending = false;
      jogDirection = direction;
      const int64_t horizon = static_cast<int64_t>(config::PULSES_PER_TABLE_REV) * 100;
      int64_t target = currentPulses() + direction * horizon;
      if (target > LONG_MAX - horizon || target < LONG_MIN + horizon) {
        sendError(request, 409, "position_limit", "Position range exhausted; stop and re-establish software zero.");
        return;
      }
      rememberOperation(operationId);
      moveToPulses(target, MotionKind::Jog, speed);
    } else {
      rememberOperation(operationId);
      setMotionLimits(speed);
    }
    jogExpiry = millis() + settings.jogLeaseMs;
    sendStatus(request, 202);
    return;
  }

  if (path == "/api/v1/jog/stop") {
    rememberOperation(operationId);
    if (motion == MotionKind::Jog) startStop();
    sendStatus(request, 202);
    return;
  }

  if (path == "/api/v1/continuous/start") {
    int direction;
    double speed;
    if (!parseDirection(doc, direction) ||
        !optionalNumber(doc, "speedDegreesPerSecond", settings.maxSpeedDps, speed) ||
        !turntable::finite(speed) ||
        speed < config::MIN_SPEED_DPS || speed > settings.maxSpeedDps) {
      sendError(request, 400, "invalid_continuous", "Provide a direction and a valid speed.");
      return;
    }
    if (!positionVerified || !isIdle() || scanActive) {
      sendError(request, 409, "motion_conflict", "Continuous rotation requires verified position and idle motion.");
      return;
    }
    const long horizon = config::PULSES_PER_TABLE_REV * 100L;
    const long current = currentPulses();
    if ((direction > 0 && current > LONG_MAX - horizon) ||
        (direction < 0 && current < LONG_MIN + horizon)) {
      sendError(request, 409, "position_limit", "Position range exhausted; re-establish software zero.");
      return;
    }
    manualStopPending = false;
    continuousDirection = direction;
    rememberOperation(operationId);
    moveToPulses(current + direction * horizon, MotionKind::Continuous, speed);
    sendStatus(request, 202);
    return;
  }

  if (path == "/api/v1/stop") {
    rememberOperation(operationId);
    if (scanActive) {
      scanSequence.requestPause();
      if (scanState == ScanState::Pausing) startStop();
      else if (motion == MotionKind::Scan) motion = MotionKind::Idle;
    } else if (motion != MotionKind::Idle) {
      startStop();
    }
    sendStatus(request, 202);
    return;
  }

  if (path == "/api/v1/scan/start") {
    if (!positionVerified || !isIdle() || scanActive) {
      sendError(request, 409, "motion_conflict", "Scan requires verified position and idle motion.");
      return;
    }
    ScanDefinition requested;
    double total = settings.defaultScanDegrees;
    if (doc.containsKey("totalDegrees") &&
        !bodyNumber(doc, "totalDegrees", total)) {
      sendError(request, 400, "invalid_scan", "totalDegrees must be finite.");
      return;
    }
    requested.totalDegrees = total;
    if (doc.containsKey("mode") && !doc["mode"].is<const char*>()) {
      sendError(request, 400, "invalid_scan", "mode must be a string.");
      return;
    }
    const char* scanMode = doc["mode"] | "views";
    if (strcmp(scanMode, "views") == 0) {
      requested.mode = ScanMode::ViewCount;
      requested.views = doc["views"] | settings.defaultScanViews;
    } else if (strcmp(scanMode, "increment") == 0) {
      requested.mode = ScanMode::Increment;
      double increment;
      if (!bodyNumber(doc, "incrementDegrees", increment)) {
        sendError(request, 400, "invalid_scan", "incrementDegrees must be finite.");
        return;
      }
      requested.incrementDegrees = increment;
    } else {
      sendError(request, 400, "invalid_scan", "mode must be views or increment.");
      return;
    }
    if (doc.containsKey("views") && !doc["views"].is<uint16_t>()) {
      sendError(request, 400, "invalid_scan", "views must be an integer.");
      return;
    }
    if ((doc.containsKey("autoAdvance") && !doc["autoAdvance"].is<bool>()) ||
        (doc.containsKey("returnToStart") && !doc["returnToStart"].is<bool>())) {
      sendError(request, 400, "invalid_scan", "autoAdvance and returnToStart must be booleans.");
      return;
    }
    double speed;
    if (!optionalNumber(doc, "speedDegreesPerSecond", settings.maxSpeedDps, speed) ||
        !turntable::validScan(requested) || !turntable::finite(speed) ||
        speed < config::MIN_SPEED_DPS || speed > settings.maxSpeedDps) {
      sendError(request, 400, "invalid_scan", "Scan values are outside documented limits.");
      return;
    }
    const int64_t finalTarget = turntable::scanTargetPulses(
        currentPulses(), requested, turntable::scanViewCount(requested) - 1);
    if (finalTarget > LONG_MAX || finalTarget < LONG_MIN) {
      sendError(request, 409, "position_limit", "Scan target exceeds supported position range.");
      return;
    }
    activeSpeedDps = speed;
    returnToStart = doc["returnToStart"] | settings.returnToStartDefault;
    autoAdvance = doc["autoAdvance"] | settings.autoAdvanceDefault;
    rememberOperation(operationId);
    scanSequence.begin(requested, currentPulses(), returnToStart, autoAdvance);
    scanMessage = "Scan accepted; the starting position is view 1.";
    setEnabled(true);
    beginScanView(0);
    sendStatus(request, 202);
    return;
  }

  if (path == "/api/v1/scan/pause") {
    if (!scanActive || scanPaused) {
      sendError(request, 409, "invalid_scan_state", "There is no running scan to pause.");
      return;
    }
    rememberOperation(operationId);
    scanSequence.requestPause();
    if (scanState == ScanState::Pausing) startStop();
    else if (motion == MotionKind::Scan) motion = MotionKind::Idle;
    sendStatus(request, 202);
    return;
  }

  if (path == "/api/v1/scan/resume") {
    if (!scanActive || !scanPaused) {
      sendError(request, 409, "invalid_scan_state", "Scan is not paused.");
      return;
    }
    rememberOperation(operationId);
    const bool resumeMotion = scanSequence.resume();
    if (resumeMotion) {
      if (scanSequence.returning) {
        moveToPulses(scanStartPulse, MotionKind::Return, activeSpeedDps);
      } else {
        beginScanView(scanView);
      }
    } else if (readyView) {
      motion = MotionKind::Idle;
    }
    sendStatus(request, 202);
    return;
  }

  if (path == "/api/v1/scan/cancel") {
    if (!scanActive) {
      sendError(request, 409, "invalid_scan_state", "No active scan to cancel.");
      return;
    }
    rememberOperation(operationId);
    const bool scanMotion = motion == MotionKind::Scan || motion == MotionKind::Return;
    scanSequence.requestCancel(scanMotion);
    if (scanMotion) {
      stepper.stop();
    } else {
      clearScan(ScanState::Cancelled, "Scan cancelled.");
    }
    sendStatus(request, 202);
    return;
  }

  if (path == "/api/v1/scan/acknowledge-view") {
    if (!scanActive || scanPaused || scanState != ScanState::Ready) {
      sendError(request, 409, "view_not_ready", "A scan view must be Ready before it can be acknowledged.");
      return;
    }
    rememberOperation(operationId);
    advanceScan();
    sendStatus(request, 202);
    return;
  }

  if (path == "/api/v1/position/zero") {
    if (!isIdle() || scanActive) {
      sendError(request, 409, "motion_active", "Stop motion before setting software zero.");
      return;
    }
    rememberOperation(operationId);
    zeroPulse = currentPulses();
    exactPositionPulses = currentPulses();
    positionVerified = true;
    sendStatus(request);
    return;
  }

  if (path == "/api/v1/position/go-zero") {
    if (!positionVerified || !isIdle() || scanActive) {
      sendError(request, 409, "position_unverified", "Go to zero requires verified position and idle motion.");
      return;
    }
    rememberOperation(operationId);
    exactPositionPulses = zeroPulse;
    moveToPulses(zeroPulse, MotionKind::Move, settings.maxSpeedDps);
    sendStatus(request, 202);
    return;
  }

  if (path == "/api/v1/motor/enable") {
    if (!isIdle()) {
      sendError(request, 409, "motion_active", "Motor enable state cannot change while moving.");
      return;
    }
    rememberOperation(operationId);
    setEnabled(true);
    sendStatus(request);
    return;
  }

  if (path == "/api/v1/motor/disable") {
    if (scanActive) {
      sendError(request, 409, "scan_active", "Pause or cancel the scan before disabling the motor.");
      return;
    }
    if (isIdle()) {
      rememberOperation(operationId);
      setEnabled(false);
      sendStatus(request);
    } else {
      rememberOperation(operationId);
      pendingDisable = true;
      startStop();
      sendStatus(request, 202);
    }
    return;
  }

  sendError(request, 404, "not_found", "Unknown API endpoint.");
}

void handlePostBody(AsyncWebServerRequest* request, uint8_t* data, size_t len,
                    size_t index, size_t total) {
  if (request->_tempObject == &bodyHandledMarker) return;
  if (total > 2048) {
    request->_tempObject = &bodyHandledMarker;
    sendError(request, 413, "body_too_large", "JSON request body exceeds 2048 bytes.");
    return;
  }
  if (index == 0) {
    request->_tempObject = malloc(total + 1);
    if (request->_tempObject == nullptr) {
      request->_tempObject = &bodyHandledMarker;
      sendError(request, 503, "request_error", "Unable to allocate request body.");
      return;
    }
  }
  auto* body = static_cast<char*>(request->_tempObject);
  if (body == nullptr) {
    request->_tempObject = &bodyHandledMarker;
    sendError(request, 500, "request_error", "Unable to allocate request body.");
    return;
  }
  memcpy(body + index, data, len);
  body[index + len] = '\0';
  if (index + len != total) return;

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, body);
  free(body);
  request->_tempObject = &bodyHandledMarker;
  if (error) {
    sendError(request, 400, "invalid_json", "Request body must contain valid JSON.");
    return;
  }
  lockController();
  dispatchCommand(request, doc);
  unlockController();
}

void loadSettings() {
  if (!preferences.begin("turntable", false)) {
    settings = Settings();
    scanMessage = "NVS unavailable; using safe defaults for this boot.";
    return;
  }
  settings.ssid = preferences.getString("ssid", config::DEFAULT_AP_SSID);
  settings.password = preferences.getString("password", config::DEFAULT_AP_PASSWORD);
  settings.maxSpeedDps = preferences.getFloat("maxSpeed", config::DEFAULT_MAX_SPEED_DPS);
  settings.accelerationDps2 = preferences.getFloat("accel", config::DEFAULT_ACCEL_DPS2);
  settings.settleMs = preferences.getUInt("settle", config::DEFAULT_SETTLE_MS);
  settings.jogLeaseMs = preferences.getUInt("jogLease", config::DEFAULT_JOG_LEASE_MS);
  settings.defaultScanDegrees = preferences.getFloat("scanDegrees", 360.0f);
  settings.defaultScanViews =
      preferences.getUShort("scanViews", config::DEFAULT_SCAN_VIEWS);
  settings.autoAdvanceDefault = preferences.getBool("autoAdvance", false);
  settings.returnToStartDefault = preferences.getBool("returnStart", false);
  if (!turntable::validApCredentials(settings.ssid.length(), settings.password.length()) ||
      !turntable::validMotionSettings(settings.maxSpeedDps,
                                      settings.accelerationDps2,
                                      settings.settleMs, settings.jogLeaseMs) ||
      !turntable::validScanDefaults(settings.defaultScanDegrees,
                                    settings.defaultScanViews)) {
    settings = Settings();
    preferences.putString("ssid", settings.ssid);
    preferences.putString("password", settings.password);
    preferences.putFloat("maxSpeed", settings.maxSpeedDps);
    preferences.putFloat("accel", settings.accelerationDps2);
    preferences.putUInt("settle", settings.settleMs);
    preferences.putUInt("jogLease", settings.jogLeaseMs);
    preferences.putFloat("scanDegrees", settings.defaultScanDegrees);
    preferences.putUShort("scanViews", settings.defaultScanViews);
    preferences.putBool("autoAdvance", settings.autoAdvanceDefault);
    preferences.putBool("returnStart", settings.returnToStartDefault);
    scanMessage = "Invalid stored settings replaced with safe defaults.";
  }
}

void registerPost(const char* path) {
  server.on(path, HTTP_POST,
            [](AsyncWebServerRequest* request) {
              if (request->_tempObject == &bodyHandledMarker) {
                request->_tempObject = nullptr;
              } else {
                sendError(request, 400, "invalid_json", "Request body must contain valid JSON.");
              }
            },
            nullptr, handlePostBody);
}

void registerPut(const char* path) {
  server.on(path, HTTP_PUT,
            [](AsyncWebServerRequest* request) {
              if (request->_tempObject == &bodyHandledMarker) {
                request->_tempObject = nullptr;
              } else {
                sendError(request, 400, "invalid_json", "Request body must contain valid JSON.");
              }
            },
            nullptr, handlePostBody);
}

void onWebSocketEvent(AsyncWebSocket*, AsyncWebSocketClient* client,
                      AwsEventType type, void*, uint8_t*, size_t) {
  if (type == WS_EVT_CONNECT) {
    lockController();
    JsonDocument doc = makeStatus();
    client->text(jsonString(doc));
    unlockController();
  }
}

void configureServer() {
  server.on("/api/v1/status", HTTP_GET, [](AsyncWebServerRequest* request) {
    lockController();
    sendStatus(request);
    unlockController();
  });
  server.on("/api/v1/settings", HTTP_GET, [](AsyncWebServerRequest* request) {
    lockController();
    JsonDocument doc;
    doc["ok"] = true;
    doc["ssid"] = settings.ssid;
    doc["passwordConfigured"] = true;
    doc["maxSpeedDegreesPerSecond"] = settings.maxSpeedDps;
    doc["accelerationDegreesPerSecondSquared"] = settings.accelerationDps2;
    doc["settleMs"] = settings.settleMs;
    doc["jogLeaseMs"] = settings.jogLeaseMs;
    doc["defaultScanDegrees"] = settings.defaultScanDegrees;
    doc["defaultScanViews"] = settings.defaultScanViews;
    doc["autoAdvanceDefault"] = settings.autoAdvanceDefault;
    doc["returnToStartDefault"] = settings.returnToStartDefault;
    doc["limits"]["maxSpeedDegreesPerSecond"] = config::MAX_SPEED_DPS;
    doc["limits"]["maxAccelerationDegreesPerSecondSquared"] = config::MAX_ACCEL_DPS2;
    request->send(200, "application/json", jsonString(doc));
    unlockController();
  });
  registerPost("/api/v1/move");
  registerPost("/api/v1/jog/start");
  registerPost("/api/v1/jog/stop");
  registerPost("/api/v1/continuous/start");
  registerPost("/api/v1/stop");
  registerPost("/api/v1/scan/start");
  registerPost("/api/v1/scan/pause");
  registerPost("/api/v1/scan/resume");
  registerPost("/api/v1/scan/cancel");
  registerPost("/api/v1/scan/acknowledge-view");
  registerPost("/api/v1/position/zero");
  registerPost("/api/v1/position/go-zero");
  registerPost("/api/v1/motor/enable");
  registerPost("/api/v1/motor/disable");
  registerPut("/api/v1/settings");

  websocket.onEvent(onWebSocketEvent);
  server.addHandler(&websocket);
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
  server.onNotFound([](AsyncWebServerRequest* request) {
    if (request->url().startsWith("/api/")) {
      sendError(request, 404, "not_found", "Unknown API endpoint.");
    } else {
      request->send(404, "text/plain", "Not found");
    }
  });
  server.begin();
}

void updateMotion() {
  const uint32_t now = millis();
  if (motion == MotionKind::Jog && !manualStopPending &&
      static_cast<int32_t>(now - jogExpiry) >= 0) {
    manualStopPending = true;
    stepper.stop();
  }
  if (!manualStopPending &&
      (motion == MotionKind::Jog || motion == MotionKind::Continuous) &&
      std::abs(stepper.distanceToGo()) < config::PULSES_PER_TABLE_REV) {
    const int direction = motion == MotionKind::Jog ? jogDirection : continuousDirection;
    const long horizon = config::PULSES_PER_TABLE_REV * 100L;
    const long current = stepper.currentPosition();
    if ((direction > 0 && current < LONG_MAX - horizon) ||
        (direction < 0 && current > LONG_MIN + horizon)) {
      stepper.moveTo(current + direction * horizon);
    } else {
      startStop();
    }
  }

  stepper.run();
  if (motion != MotionKind::Idle && stepper.distanceToGo() == 0) {
    const MotionKind completed = motion;
    motion = MotionKind::Idle;
    manualStopPending = false;
    if (completed != MotionKind::Move ||
        currentPulses() != expectedTargetPulse) {
      exactPositionPulses = currentPulses();
    }
    if (completed == MotionKind::Scan) {
      if (scanCancelPending) {
        scanSequence.cancelStopped();
        clearScan(ScanState::Cancelled, "Scan cancelled.");
      } else if (scanPausePending) {
        const bool reachedTarget = currentPulses() == scanTargetPulse;
        scanSequence.stoppedAfterPause(reachedTarget);
        if (reachedTarget) {
          settleDeadline = now + settings.settleMs;
        }
      } else {
        scanSequence.reachedTarget();
        settleDeadline = now + settings.settleMs;
      }
    } else if (completed == MotionKind::Return) {
      if (scanCancelPending) {
        scanSequence.cancelStopped();
        clearScan(ScanState::Cancelled, "Scan cancelled.");
      } else if (scanPausePending) {
        scanSequence.stoppedAfterPause(currentPulses() == scanTargetPulse);
      } else {
        clearScan(ScanState::Complete, "Scan complete and returned to starting position.");
      }
    }
    if (pendingDisable) {
      pendingDisable = false;
      setEnabled(false);
    }
  }

  if (scanActive && scanState == ScanState::Settling &&
      static_cast<int32_t>(now - settleDeadline) >= 0) {
    scanSequence.settlingComplete();
    scanMessage = "View is Ready for manual image capture.";
    if (autoAdvance && !scanPaused) advanceScan();
  }
}
}  // namespace

void setup() {
  digitalWrite(config::STEP_PIN, LOW);
  digitalWrite(config::DIR_PIN, LOW);
  digitalWrite(config::ENABLE_PIN, config::ENABLE_ACTIVE_LOW ? HIGH : LOW);
  pinMode(config::STEP_PIN, OUTPUT);
  pinMode(config::DIR_PIN, OUTPUT);
  pinMode(config::ENABLE_PIN, OUTPUT);

  Serial.begin(115200);
  controllerMutex = xSemaphoreCreateMutex();
  if (controllerMutex == nullptr) {
    Serial.println("Failed to create controller mutex; outputs remain disabled.");
    while (true) delay(1000);
  }
  loadSettings();
  stepper.setEnablePin(config::ENABLE_PIN);
  stepper.setPinsInverted(false, false, config::ENABLE_ACTIVE_LOW);
  stepper.setMaxSpeed(settings.maxSpeedDps * config::PULSES_PER_TABLE_REV / 360.0f);
  stepper.setAcceleration(settings.accelerationDps2 * config::PULSES_PER_TABLE_REV / 360.0f);
  stepper.setCurrentPosition(0);
  exactPositionPulses = 0.0L;
  stepper.disableOutputs();
  motorEnabled = false;
  positionVerified = false;

  if (!LittleFS.begin(false)) {
    Serial.println("LittleFS mount failed; upload the filesystem image.");
  }
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(settings.ssid.c_str(), settings.password.c_str())) {
    Serial.println("Failed to start Wi-Fi access point.");
  } else {
    Serial.print("Turntable AP address: ");
    Serial.println(WiFi.softAPIP());
  }
  configureServer();
}

void loop() {
  lockController();
  updateMotion();
  unlockController();
  websocket.cleanupClients();
  const uint32_t now = millis();
  if (now - lastStatus >= config::STATUS_INTERVAL_MS) {
    lastStatus = now;
    broadcastStatus();
  }
}
