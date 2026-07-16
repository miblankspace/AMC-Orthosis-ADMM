#pragma once

#include <Arduino.h>

class MotionController;

// Holds the latest parsed motor telemetry and prints it in a stable format.
class Status {
 public:
  Status();

  void updateFromServoStatusFrame(const uint8_t* data, uint8_t length);
  bool hasStatus() const;

  float getPositionDeg() const;
  float getMeasuredErpm() const;
  int8_t getTemperatureC() const;
  uint8_t getError() const;
  int16_t getRawCurrent() const;
  float getCurrentAmps() const;
  float getEstimatedTorqueNm() const;

  void print(const MotionController& motion) const;

 private:
  bool hasStatus_;
  float positionDeg_;
  float measuredErpm_;
  int8_t temperatureC_;
  uint8_t error_;
  int16_t rawCurrent_;
};
