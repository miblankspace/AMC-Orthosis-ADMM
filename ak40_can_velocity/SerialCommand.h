#pragma once

#include <Arduino.h>

class MotionController;

// Parses serial commands and dispatches motion-intent requests.
class SerialCommand {
 public:
  explicit SerialCommand(MotionController& motion);

  void update();

  bool consumeStatusRequest();
  bool consumeReconnectRequest();
  bool consumeEmergencyStopRequest();

 private:
  void handleLine(const String& line);
  int32_t parseValue(const String& line) const;

  MotionController& motion_;
  bool statusRequested_;
  bool reconnectRequested_;
  bool emergencyStopRequested_;
};
