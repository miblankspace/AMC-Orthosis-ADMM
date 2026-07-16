#include "Config.h"
#include "MotionController.h"
#include "MotorCAN.h"
#include "SerialCommand.h"

MotorCAN motor(Config::kCanCsPin, Config::kCanIntPin, Config::kMotorId);
MotionController motion(Config::kMaxErpm, Config::kAccelStep, Config::kStopStep,
                        Config::kEmergencyStep, Config::kMotionUpdatePeriodMs);
SerialCommand serialCommand(motion);

uint32_t lastCommandMs = 0;
uint32_t lastStatusPrintMs = 0;
bool emergencyPassiveSent = false;

static void printStatusIfAvailable() {
  if (motor.getStatus().hasStatus()) {
    motor.getStatus().print(motion);
  } else {
    Serial.println("No 0x29 status yet");
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("=== AK40-10 velocity mode ===");
  if (!motor.begin()) {
    Serial.println("MCP2515 init failed");
    while (true) {
      delay(1000);
    }
  }

  if (!motor.servoConnect()) {
    Serial.println("No motor yet - use h after power-on");
  }

  Serial.print("Max |ERPM|=");
  Serial.println(Config::kMaxErpm);
  Serial.println("Commands: v <erpm> | s=stop | e=emergency | i=status | h=reconnect");

  if (Config::kAutoStart) {
    motion.moveUp(Config::kDefaultErpm);
  } else {
    Serial.print("AUTO_START off - send e.g. v ");
    Serial.println(Config::kDefaultErpm);
  }
}

void loop() {
  motor.poll();
  serialCommand.update();

  if (serialCommand.consumeReconnectRequest()) {
    Serial.println(motor.reconnect() ? "Reconnected" : "Reconnect failed");
  }

  if (serialCommand.consumeEmergencyStopRequest()) {
    motion.emergencyStop();
    Serial.println("Emergency stop: motor released passive");
  }

  if (motor.getStatus().getError() != 0) {
    motion.emergencyStop();
  }

  motion.update();

  if (motion.isEmergencyStopActive()) {
    if (!emergencyPassiveSent) {
      if (!motor.servoSetDuty(0.0f)) {
        Serial.println("CAN TX failed");
      }
      emergencyPassiveSent = true;
    }
  } else if (millis() - lastCommandMs >= Config::kCommandPeriodMs) {
    if (!motor.servoSetRPM(motion.getCurrentERPM())) {
      Serial.println("CAN TX failed");
    }
    lastCommandMs = millis();
    emergencyPassiveSent = false;
  }

  const bool shouldPrintStatus =
      serialCommand.consumeStatusRequest() ||
      (motor.getStatus().hasStatus() &&
       millis() - lastStatusPrintMs >= Config::kStatusPeriodMs);
  if (shouldPrintStatus) {
    printStatusIfAvailable();
    lastStatusPrintMs = millis();
  }
}
