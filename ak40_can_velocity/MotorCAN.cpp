#include "MotorCAN.h"

#include <SPI.h>

MotorCAN::MotorCAN(uint8_t csPin, uint8_t intPin, uint8_t motorId)
    : csPin_(csPin),
      intPin_(intPin),
      motorId_(motorId),
      mcp2515_(csPin) {}

bool MotorCAN::begin() {
  pinMode(intPin_, INPUT);
  SPI.begin();
  mcp2515_.reset();

  if (mcp2515_.setBitrate(CAN_1000KBPS, MCP_16MHZ) != MCP2515::ERROR_OK) {
    return false;
  }

  return mcp2515_.setNormalMode() == MCP2515::ERROR_OK;
}

void MotorCAN::poll() {
  while (readCanFrame(0)) {
    processServoFrame();
  }
}

bool MotorCAN::servoConnect() {
  uint32_t end = millis() + 800;
  while (millis() < end) {
    if (readCanFrame(25) && processServoFrame()) {
      return true;
    }
  }

  const uint8_t zeros[8] = {0};
  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    sendExt(servoExtId(0x2C), zeros, 8);
    end = millis() + 1500;
    while (millis() < end) {
      if (!readCanFrame(50)) {
        continue;
      }

      if (isHandshakeReply(rxFrame_) || processServoFrame()) {
        return true;
      }
    }
  }

  return status_.hasStatus();
}

bool MotorCAN::reconnect() { return servoConnect(); }

bool MotorCAN::servoSetRPM(int32_t erpm) {
  uint8_t payload[4];
  appendI32BE(payload, erpm);
  return sendExt(servoExtId(3), payload, 4);
}

bool MotorCAN::servoSetDuty(float duty) {
  uint8_t payload[4];
  appendI32BE(payload, (int32_t)(duty * 100000.0f));
  return sendExt(servoExtId(0), payload, 4);
}

const Status& MotorCAN::getStatus() const { return status_; }

Status& MotorCAN::getStatus() { return status_; }

uint32_t MotorCAN::servoExtId(uint8_t functionId) const {
  return ((uint32_t)functionId << 8) | motorId_;
}

bool MotorCAN::sendExt(uint32_t id29, const uint8_t* data, uint8_t len) {
  can_frame tx;
  tx.can_id = (id29 & CAN_EFF_MASK) | CAN_EFF_FLAG;
  tx.can_dlc = len;
  for (uint8_t i = 0; i < len; ++i) {
    tx.data[i] = data[i];
  }
  return mcp2515_.sendMessage(&tx) == MCP2515::ERROR_OK;
}

bool MotorCAN::readCanFrame(uint32_t timeoutMs) {
  if (timeoutMs == 0) {
    return mcp2515_.readMessage(&rxFrame_) == MCP2515::ERROR_OK;
  }

  const uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    if (mcp2515_.readMessage(&rxFrame_) == MCP2515::ERROR_OK) {
      return true;
    }
    delay(1);
  }
  return false;
}

bool MotorCAN::isHandshakeReply(const can_frame& frame) const {
  return frame.can_dlc >= 4 && frame.data[0] == 0xFA && frame.data[1] == 0xFB &&
         frame.data[2] == 0xFC && frame.data[3] == 0xFD;
}

bool MotorCAN::processServoFrame() {
  if ((rxFrame_.can_id & CAN_EFF_FLAG) == 0) {
    return isHandshakeReply(rxFrame_);
  }

  const uint32_t id = rxFrame_.can_id & CAN_EFF_MASK;
  const uint8_t func = (uint8_t)(id >> 8);
  const uint8_t node = (uint8_t)(id & 0xFF);
  if (node != motorId_) {
    return false;
  }

  if (func == 0x2C) {
    sendHandshakeReply();
    return true;
  }

  if (func == 0x29 && rxFrame_.can_dlc == 8) {
    status_.updateFromServoStatusFrame(rxFrame_.data, rxFrame_.can_dlc);
    return true;
  }

  return isHandshakeReply(rxFrame_);
}

void MotorCAN::sendHandshakeReply() {
  const uint8_t reply[8] = {0xFA, 0xFB, 0xFC, 0xFD, 0, 0, 0, 0};
  sendExt(servoExtId(0x2C), reply, 8);
}

void MotorCAN::appendI32BE(uint8_t* buf, int32_t value) const {
  buf[0] = (uint8_t)(value >> 24);
  buf[1] = (uint8_t)(value >> 16);
  buf[2] = (uint8_t)(value >> 8);
  buf[3] = (uint8_t)value;
}
