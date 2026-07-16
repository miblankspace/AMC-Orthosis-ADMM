#pragma once

#include <Arduino.h>
#include <mcp2515.h>

#include "Status.h"

// Low-level CAN transport and AK40 servo CAN protocol adapter.
class MotorCAN {
 public:
  MotorCAN(uint8_t csPin, uint8_t intPin, uint8_t motorId);

  bool begin();
  void poll();
  bool servoConnect();
  bool reconnect();
  bool servoSetRPM(int32_t erpm);
  bool servoSetDuty(float duty);

  const Status& getStatus() const;
  Status& getStatus();

 private:
  uint32_t servoExtId(uint8_t functionId) const;
  bool sendExt(uint32_t id29, const uint8_t* data, uint8_t len);
  bool readCanFrame(uint32_t timeoutMs);
  bool isHandshakeReply(const can_frame& frame) const;
  bool processServoFrame();
  void sendHandshakeReply();
  void appendI32BE(uint8_t* buf, int32_t value) const;

  uint8_t csPin_;
  uint8_t intPin_;
  uint8_t motorId_;
  MCP2515 mcp2515_;
  can_frame rxFrame_;
  Status status_;
};
