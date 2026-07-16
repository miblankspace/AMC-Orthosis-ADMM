#include "MotionController.h"
#include <stdlib.h> // abs()
#include "esp_timer.h"

static uint32_t millis()
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

MotionController::MotionController(int32_t maxErpm, int32_t accelStep,
                                    int32_t stopStep, int32_t emergencyStep,
                                    uint32_t updatePeriodMs)
    : maxErpm_(maxErpm),
      accelStep_(accelStep),
      stopStep_(stopStep),
      emergencyStep_(emergencyStep),
      updatePeriodMs_(updatePeriodMs),
      lastUpdateMs_(0),
      currentErpm_(0),
      targetErpm_(0),
      state_(IDLE) {}

void MotionController::moveUp(int32_t erpm)
{
    targetErpm_ = clampErpm(abs(erpm));
    state_ = (targetErpm_ == 0) ? IDLE : MOVING_UP;
}

void MotionController::moveDown(int32_t erpm)
{
    targetErpm_ = -clampErpm(abs(erpm));
    state_ = (targetErpm_ == 0) ? IDLE : MOVING_DOWN;
}

void MotionController::stop()
{
    targetErpm_ = 0;
    if (currentErpm_ == 0) {
        state_ = IDLE;
        return;
    }
    state_ = STOPPING;
}

void MotionController::emergencyStop()
{
    targetErpm_ = 0;
    currentErpm_ = 0;
    state_ = EMERGENCY_STOP;
}

void MotionController::update()
{
    const uint32_t now = millis();
    if (now - lastUpdateMs_ < updatePeriodMs_) {
        return;
    }
    lastUpdateMs_ = now;

    if (state_ == MOVING_UP || state_ == MOVING_DOWN) {
        currentErpm_ = rampToward(currentErpm_, targetErpm_, accelStep_);
        return;
    }

    if (state_ == STOPPING) {
        currentErpm_ = rampToward(currentErpm_, 0, stopStep_);
        transitionToIdleIfStopped();
        return;
    }

    if (state_ == EMERGENCY_STOP) {
        return;
    }

    currentErpm_ = 0;
}

void MotionController::extend(int32_t erpm) { moveUp(erpm); }
void MotionController::flex(int32_t erpm) { moveDown(erpm); }

int32_t MotionController::getCurrentERPM() const { return currentErpm_; }
int32_t MotionController::getTargetERPM() const { return targetErpm_; }
MotionState MotionController::getState() const { return state_; }
bool MotionController::isEmergencyStopActive() const { return state_ == EMERGENCY_STOP; }

const char* MotionController::getStateName() const
{
    switch (state_) {
        case IDLE:           return "IDLE";
        case MOVING_UP:       return "MOVING_UP";
        case MOVING_DOWN:     return "MOVING_DOWN";
        case STOPPING:        return "STOPPING";
        case EMERGENCY_STOP:  return "EMERGENCY_STOP";
        default:              return "UNKNOWN";
    }
}

int32_t MotionController::clampErpm(int32_t erpm) const
{
    if (erpm > maxErpm_) return maxErpm_;
    if (erpm < -maxErpm_) return -maxErpm_;
    return erpm;
}

int32_t MotionController::rampToward(int32_t current, int32_t target, int32_t step) const
{
    if (current < target) {
        const int32_t next = current + step;
        return (next > target) ? target : next;
    }
    if (current > target) {
        const int32_t next = current - step;
        return (next < target) ? target : next;
    }
    return target;
}

void MotionController::transitionToIdleIfStopped()
{
    if (currentErpm_ == 0) {
        state_ = IDLE;
    }
}