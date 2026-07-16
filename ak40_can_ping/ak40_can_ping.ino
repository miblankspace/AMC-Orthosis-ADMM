/*
 * AK40-10 CAN ping test
 * Hardware: Arduino Uno R4 + Joy-IT SBC-CAN01 (MCP2515, 16 MHz)
 *
 * Install library: "mcp2515" by autowp  (Arduino Library Manager)
 *
 * Wiring (SBC-CAN01 -> Uno R4):
 *   VCC, VCC1 -> 5V | GND -> GND | CS -> D10 | SI -> D11 | SO -> D12 | SCK -> D13 | INT -> D2
 * Motor CAN: CANH -> White | CANL -> Blue
 * Enable 120 ohm termination jumper P1 on the module.
 */

#include <SPI.h>
#include <mcp2515.h>
#include <math.h>

// --- pins / motor config ---
static const uint8_t CAN_CS_PIN  = 10;
static const uint8_t CAN_INT_PIN = 2;
static const uint8_t MOTOR_ID    = 69;   // factory default from AppParams

// 0 = servo firmware (factory), 1 = MIT firmware
static const uint8_t MOTOR_MODE = 0;

// --- MCP2515 ---
MCP2515 mcp2515(CAN_CS_PIN);
struct can_frame rxFrame;

// --- helpers ---
static uint32_t servoExtId(uint8_t functionId) {
  return ((uint32_t)functionId << 8) | MOTOR_ID;
}

static bool sendExt(uint32_t id29, const uint8_t *data, uint8_t len) {
  struct can_frame tx;
  tx.can_id  = (id29 & CAN_EFF_MASK) | CAN_EFF_FLAG;
  tx.can_dlc = len;
  for (uint8_t i = 0; i < len; i++) {
    tx.data[i] = data[i];
  }
  return mcp2515.sendMessage(&tx) == MCP2515::ERROR_OK;
}

static bool sendStd(uint16_t stdId, const uint8_t *data, uint8_t len) {
  struct can_frame tx;
  tx.can_id  = stdId & CAN_SFF_MASK;
  tx.can_dlc = len;
  for (uint8_t i = 0; i < len; i++) {
    tx.data[i] = data[i];
  }
  return mcp2515.sendMessage(&tx) == MCP2515::ERROR_OK;
}

static void appendI32BE(uint8_t *buf, int32_t v) {
  buf[0] = (uint8_t)(v >> 24);
  buf[1] = (uint8_t)(v >> 16);
  buf[2] = (uint8_t)(v >> 8);
  buf[3] = (uint8_t)v;
}

static bool readCanFrame(uint32_t timeoutMs) {
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    if (mcp2515.readMessage(&rxFrame) == MCP2515::ERROR_OK) {
      return true;
    }
    delay(1);
  }
  return false;
}

static void printFrame(const char *tag) {
  const bool ext = (rxFrame.can_id & CAN_EFF_FLAG) != 0;
  const uint32_t id = ext ? (rxFrame.can_id & CAN_EFF_MASK) : (rxFrame.can_id & CAN_SFF_MASK);

  Serial.print(tag);
  Serial.print(ext ? " EXT 0x" : " STD 0x");
  Serial.print(id, HEX);
  Serial.print(" DLC=");
  Serial.print(rxFrame.can_dlc);
  Serial.print(" [");
  for (uint8_t i = 0; i < rxFrame.can_dlc; i++) {
    if (i) Serial.print(' ');
    if (rxFrame.data[i] < 0x10) Serial.print('0');
    Serial.print(rxFrame.data[i], HEX);
  }
  Serial.println(']');
}

static bool isHandshakeReply() {
  return rxFrame.can_dlc >= 4
      && rxFrame.data[0] == 0xFA
      && rxFrame.data[1] == 0xFB
      && rxFrame.data[2] == 0xFC
      && rxFrame.data[3] == 0xFD;
}

static float wrap360(float deg) {
  float x = fmodf(deg, 360.0f);
  return (x < 0.0f) ? x + 360.0f : x;
}

static void parseServoStatus() {
  int16_t posRaw = (int16_t)((rxFrame.data[0] << 8) | rxFrame.data[1]);
  int16_t spdRaw = (int16_t)((rxFrame.data[2] << 8) | rxFrame.data[3]);
  int16_t curRaw = (int16_t)((rxFrame.data[4] << 8) | rxFrame.data[5]);
  int8_t tempC   = (int8_t)rxFrame.data[6];
  uint8_t err    = rxFrame.data[7];
  const float posDeg = posRaw * 0.1f;

  Serial.print("  pos=");
  Serial.print(posDeg, 1);
  Serial.print(" deg abs, wrap=");
  Serial.print(wrap360(posDeg), 1);
  Serial.print(" deg  erpm=");
  Serial.print(spdRaw * 10);
  Serial.print("  cur=");
  Serial.print(curRaw * 0.01f, 2);
  Serial.print(" A  drv_temp=");
  Serial.print(tempC);
  Serial.print(" C  err=");
  Serial.println(err);
}

static bool servoHandshake() {
  Serial.println("Servo: sending handshake (func 0x2C)...");

  const uint8_t zeros[8] = {0};
  if (!sendExt(servoExtId(0x2C), zeros, 8)) {
    Serial.println("  TX failed");
    return false;
  }

  uint32_t deadline = millis() + 2000;
  while (millis() < deadline) {
    if (!readCanFrame(100)) {
      continue;
    }
    printFrame("  RX");

    if (isHandshakeReply()) {
      Serial.println("  OK: motor replied FA FB FC FD");
      return true;
    }

  }

  Serial.println("  No handshake reply (check power, CAN wiring, ID, 1 Mbps, termination)");
  return false;
}

static bool servoSetRpmZero() {
  uint8_t payload[4];
  appendI32BE(payload, 0);
  Serial.println("Servo: SET_RPM = 0");
  return sendExt(servoExtId(3), payload, 4);
}

static bool servoListenStatus(uint32_t durationMs) {
  Serial.println("Listening for servo status frames (func 0x29)...");
  uint32_t end = millis() + durationMs;
  bool gotStatus = false;

  while (millis() < end) {
    if (!readCanFrame(100)) {
      continue;
    }

    const bool ext = (rxFrame.can_id & CAN_EFF_FLAG) != 0;
    if (!ext) {
      printFrame("RX");
      continue;
    }

    const uint32_t id = rxFrame.can_id & CAN_EFF_MASK;
    const uint8_t func = (uint8_t)(id >> 8);
    const uint8_t node = (uint8_t)(id & 0xFF);

    if (node != MOTOR_ID) {
      continue;
    }

    if (func == 0x29 && rxFrame.can_dlc == 8) {
      printFrame("RX");
      parseServoStatus();
      gotStatus = true;
    } else if (func == 0x09) {
      printFrame("RX");
      Serial.println("  Motor in jump/boot state (0x09)");
    } else if (func == 0x2C) {
      printFrame("RX");
      Serial.println("  Motor sent 0x2C; replying FA FB FC FD");
      const uint8_t reply[8] = {0xFA, 0xFB, 0xFC, 0xFD, 0, 0, 0, 0};
      sendExt(servoExtId(0x2C), reply, 8);
    } else {
      printFrame("RX");
    }
  }

  if (!gotStatus) {
    Serial.println("  No 0x29 status yet (send_can_status may be disabled in motor config).");
  }
  return gotStatus;
}

static bool mitEnterMode() {
  Serial.println("MIT: enter motor mode (0xFF..0xFC)...");
  const uint8_t cmd[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
  if (!sendStd(MOTOR_ID, cmd, 8)) {
    Serial.println("  TX failed");
    return false;
  }

  uint32_t deadline = millis() + 2000;
  while (millis() < deadline) {
    if (!readCanFrame(100)) {
      continue;
    }
    printFrame("  RX");

    const bool ext = (rxFrame.can_id & CAN_EFF_FLAG) != 0;
    if (!ext && (rxFrame.can_id & CAN_SFF_MASK) == MOTOR_ID && rxFrame.can_dlc >= 6) {
      Serial.println("  OK: MIT status frame received");
      return true;
    }
  }

  Serial.println("  No MIT reply (motor may be on servo firmware, not MIT)");
  return false;
}

static void initCan() {
  pinMode(CAN_INT_PIN, INPUT);

  SPI.begin();
  mcp2515.reset();

  if (mcp2515.setBitrate(CAN_1000KBPS, MCP_16MHZ) != MCP2515::ERROR_OK) {
    Serial.println("ERROR: MCP2515 bitrate init failed");
    Serial.println("Check CS=D10, SPI pins, 5V on VCC+VCC1, 16 MHz crystal setting");
    while (true) {
      delay(1000);
    }
  }

  if (mcp2515.setNormalMode() != MCP2515::ERROR_OK) {
    Serial.println("ERROR: MCP2515 normal mode failed");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("MCP2515 OK @ 1000 kbps, 16 MHz");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("=== AK40-10 CAN ping (Uno R4 + SBC-CAN01) ===");
  Serial.print("Motor ID: ");
  Serial.println(MOTOR_ID);

  initCan();

  bool ok = false;

  if (MOTOR_MODE == 0) {
    ok = servoHandshake();
    if (ok) {
      servoSetRpmZero();
      ok = servoListenStatus(3000);
    } else {
      Serial.println("Still listening 3 s in case motor initiates handshake...");
      servoListenStatus(3000);
    }
  } else {
    ok = mitEnterMode();
  }

  Serial.println();
  if (ok) {
    Serial.println("RESULT: motor responded on CAN.");
  } else {
    Serial.println("RESULT: no clear motor response.");
    Serial.println("Tips:");
    Serial.println("  - Motor powered? (separate supply on XT30)");
    Serial.println("  - P1 termination enabled on SBC-CAN01?");
    Serial.println("  - CANH/CANL not swapped?");
    Serial.println("  - Correct firmware mode? Set MOTOR_MODE 0=servo, 1=MIT");
    Serial.println("  - CAN ID 69 matches motor config?");
  }
}

void loop() {
  if (readCanFrame(200)) {
    printFrame("RX");
    if (MOTOR_MODE == 0) {
      const uint32_t id = rxFrame.can_id & CAN_EFF_MASK;
      if ((id >> 8) == 0x29 && (uint8_t)(id & 0xFF) == MOTOR_ID) {
        parseServoStatus();
      }
    }
  }
}
