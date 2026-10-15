#pragma once

#include "motion_core.h"

namespace turntable {

enum class ScanState {
  Idle,
  Moving,
  Settling,
  Ready,
  Pausing,
  Paused,
  Returning,
  Canceling,
  Complete,
  Cancelled,
  Failed
};

class ScanSequence {
 public:
  ScanDefinition definition;
  ScanState state = ScanState::Idle;
  bool active = false;
  bool paused = false;
  bool pausePending = false;
  bool cancelPending = false;
  bool returnToStart = false;
  bool autoAdvance = false;
  bool ready = false;
  bool settlingWhilePaused = false;
  bool viewsComplete = false;
  bool returning = false;
  uint16_t view = 0;
  uint16_t viewCount = 0;
  int64_t startPulse = 0;
  int64_t targetPulse = 0;

  bool begin(const ScanDefinition& next, int64_t start, bool shouldReturn,
             bool shouldAutoAdvance) {
    if (!validScan(next)) return false;
    definition = next;
    startPulse = start;
    targetPulse = start;
    viewCount = scanViewCount(definition);
    view = 0;
    active = true;
    paused = false;
    pausePending = false;
    cancelPending = false;
    returnToStart = shouldReturn;
    autoAdvance = shouldAutoAdvance;
    ready = false;
    settlingWhilePaused = false;
    viewsComplete = false;
    returning = false;
    state = ScanState::Moving;
    return true;
  }

  void beginView(uint16_t index) {
    view = index;
    targetPulse = scanTargetPulses(startPulse, definition, index);
    ready = false;
    settlingWhilePaused = false;
    state = ScanState::Moving;
  }

  void reachedTarget() {
    state = ScanState::Settling;
    settlingWhilePaused = false;
  }

  bool requestPause() {
    if (!active || paused || (viewsComplete && state != ScanState::Returning)) return false;
    if (state == ScanState::Ready) {
      paused = true;
      state = ScanState::Paused;
    } else if (state == ScanState::Settling) {
      paused = true;
      settlingWhilePaused = true;
      state = ScanState::Paused;
    } else if (state == ScanState::Moving || state == ScanState::Returning ||
               state == ScanState::Pausing) {
      returning = state == ScanState::Returning || returning;
      pausePending = true;
      state = ScanState::Pausing;
    } else {
      return false;
    }
    return true;
  }

  void stoppedAfterPause(bool reachedViewTarget) {
    pausePending = false;
    paused = true;
    if (returning) {
      state = ScanState::Paused;
      return;
    }
    if (reachedViewTarget) {
      reachedTarget();
      settlingWhilePaused = true;
    } else {
      state = ScanState::Paused;
    }
  }

  bool settlingComplete() {
    if (!active || (state != ScanState::Settling &&
                    !(state == ScanState::Paused && settlingWhilePaused))) {
      return false;
    }
    ready = true;
    settlingWhilePaused = false;
    state = paused ? ScanState::Paused : ScanState::Ready;
    return true;
  }

  bool resume() {
    if (!active || !paused) return false;
    paused = false;
    if (ready) {
      state = ScanState::Ready;
      return false;
    }
    if (returning) {
      state = ScanState::Returning;
      return true;
    }
    if (settlingWhilePaused) {
      state = ScanState::Settling;
      return false;
    }
    state = ScanState::Moving;
    return true;
  }

  enum class AcknowledgeResult { Invalid, Advanced, ViewsComplete };

  AcknowledgeResult acknowledge() {
    if (!active || paused || state != ScanState::Ready) {
      return AcknowledgeResult::Invalid;
    }
    ready = false;
    if (view + 1 >= viewCount) {
      viewsComplete = true;
      return AcknowledgeResult::ViewsComplete;
    }
    ++view;
    targetPulse = scanTargetPulses(startPulse, definition, view);
    state = ScanState::Moving;
    return AcknowledgeResult::Advanced;
  }

  bool requestCancel(bool moving) {
    if (!active) return false;
    if (moving) {
      cancelPending = true;
      pausePending = false;
      state = ScanState::Canceling;
    } else {
      finish(ScanState::Cancelled);
    }
    return true;
  }

  void cancelStopped() { finish(ScanState::Cancelled); }

  void beginReturn() {
    returning = true;
    targetPulse = startPulse;
    state = ScanState::Returning;
  }

  void finish(ScanState finalState) {
    active = false;
    paused = false;
    pausePending = false;
    cancelPending = false;
    ready = false;
    settlingWhilePaused = false;
    returning = false;
    state = finalState;
  }
};

}  // namespace turntable
