#include "Status.h"

#include "MotionController.h"

Status::Status()
    : hasStatus_(false),
      positionDeg_(0.0f),
      measuredErpm_(0.0f),
      temperatureC_(0),
      error_(0),
      rawCurrent_(0) {}

void Status::updateFromServoStatusFrame(const uint8_t* data, uint8_t length) {
  if (length < 8) {
    return;
  }

  const int16_t posRaw = (int16_t)((data[0] << 8) | data[1]);
  const int16_t spdRaw = (int16_t)((data[2] << 8) | data[3]);

  rawCurrent_ = (int16_t)((data[4] << 8) | data[5]);
  positionDeg_ = posRaw * 0.1f;
  measuredErpm_ = spdRaw * 10.0f;
  temperatureC_ = (int8_t)data[6];
  error_ = data[7];
  hasStatus_ = true;
}

bool Status::hasStatus() const { return hasStatus_; }

float Status::getPositionDeg() const { return positionDeg_; }

float Status::getMeasuredErpm() const { return measuredErpm_; }

int8_t Status::getTemperatureC() const { return temperatureC_; }

uint8_t Status::getError() const { return error_; }

int16_t Status::getRawCurrent() const { return rawCurrent_; }

float Status::getCurrentAmps() const { return rawCurrent_ * 0.01f; }

float Status::getEstimatedTorqueNm() const {
  // Servo CAN status reports current in the range -60..60 A.
  // For AK40-10, the workspace guide documents a torque range of -5..5 N*m.
  static constexpr float kTorquePerAmp = 5.0f / 60.0f;
  return getCurrentAmps() * kTorquePerAmp;
}

void Status::print(const MotionController& motion) const {
  Serial.print("state=");
  Serial.print(motion.getStateName());
  Serial.print("  target_erpm=");
  Serial.print(motion.getTargetERPM());
  Serial.print("  current_erpm=");
  Serial.print(motion.getCurrentERPM());
  Serial.print("  measured_erpm=");
  Serial.print(measuredErpm_, 0);
  Serial.print("  pos=");
  Serial.print(positionDeg_, 1);
  Serial.print("  temp=");
  Serial.print(temperatureC_);
  Serial.print("C  err=");
  Serial.print(error_);
  Serial.print("  current=");
  Serial.print(getCurrentAmps(), 2);
  Serial.print("A");
  Serial.print("  current_raw=");
  Serial.print(rawCurrent_);
  Serial.print("  torque_est=");
  Serial.print(getEstimatedTorqueNm(), 2);
  Serial.print("Nm");
  Serial.println();
}
