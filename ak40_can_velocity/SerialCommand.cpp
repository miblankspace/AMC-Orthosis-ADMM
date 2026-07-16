#include "SerialCommand.h"

#include "MotionController.h"

SerialCommand::SerialCommand(MotionController& motion)
    : motion_(motion),
      statusRequested_(false),
      reconnectRequested_(false),
      emergencyStopRequested_(false) {}

void SerialCommand::update() {
  if (!Serial.available()) {
    return;
  }

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) {
    return;
  }

  handleLine(line);
}

bool SerialCommand::consumeStatusRequest() {
  const bool requested = statusRequested_;
  statusRequested_ = false;
  return requested;
}

bool SerialCommand::consumeReconnectRequest() {
  const bool requested = reconnectRequested_;
  reconnectRequested_ = false;
  return requested;
}

bool SerialCommand::consumeEmergencyStopRequest() {
  const bool requested = emergencyStopRequested_;
  emergencyStopRequested_ = false;
  return requested;
}

void SerialCommand::handleLine(const String& line) {
  if (line == "s") {
    motion_.stop();
    return;
  }

  if (line == "i") {
    statusRequested_ = true;
    return;
  }

  if (line == "h") {
    reconnectRequested_ = true;
    return;
  }

  if (line == "e") {
    emergencyStopRequested_ = true;
    return;
  }

  if (!line.startsWith("v")) {
    Serial.println("Use: v <erpm> | s | e | i | h");
    return;
  }

  const int32_t erpm = parseValue(line);
  if (erpm > 0) {
    motion_.moveUp(erpm);
    return;
  }
  if (erpm < 0) {
    motion_.moveDown(-erpm);
    return;
  }

  motion_.stop();
}

int32_t SerialCommand::parseValue(const String& line) const {
  return (int32_t)line.substring(1).toInt();
}
