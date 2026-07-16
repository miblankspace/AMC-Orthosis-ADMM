#pragma once

#include <Arduino.h>

enum MotionState {
  IDLE,
  MOVING_UP,
  MOVING_DOWN,
  STOPPING,
  EMERGENCY_STOP
};

// High-level motion state machine with smooth ERPM ramping.
class MotionController {
 public:
  MotionController(int32_t maxErpm, int32_t accelStep, int32_t stopStep,
                   int32_t emergencyStep, uint32_t updatePeriodMs);

  void moveUp(int32_t erpm);
  void moveDown(int32_t erpm);
  void stop();
  void emergencyStop();
  void update();

  void extend(int32_t erpm);
  void flex(int32_t erpm);

  int32_t getCurrentERPM() const;
  int32_t getTargetERPM() const;
  MotionState getState() const;
  const char* getStateName() const;
  bool isEmergencyStopActive() const;

 private:
  int32_t clampErpm(int32_t erpm) const;
  int32_t rampToward(int32_t current, int32_t target, int32_t step) const;
  void transitionToIdleIfStopped();

  int32_t maxErpm_;
  int32_t accelStep_;
  int32_t stopStep_;
  int32_t emergencyStep_;
  uint32_t updatePeriodMs_;
  uint32_t lastUpdateMs_;
  int32_t currentErpm_;
  int32_t targetErpm_;
  MotionState state_;
};
