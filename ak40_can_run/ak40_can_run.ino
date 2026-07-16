/*
 * AK40-10 CAN motion demo
 * Arduino Uno R4 + Joy-IT SBC-CAN01 (MCP2515, 16 MHz)
 *
 * Library: "mcp2515" by autowp
 *
 * Serial @ 115200:
 *   p <deg>   — move to absolute angle (degrees)
 *   r <deg>   — move relative to current position
 *   z         — set current position as zero (temporary origin)
 *   s         — stop (RPM = 0)
 *   h         — re-handshake
 *
 * Position from 0x29 status uses int16 * 0.1 deg (CubeMars manual).
 * Example: [37 22 ...] -> 0x3722 -> 1411.4 deg absolute.
 */

#include <SPI.h>
#include <mcp2515.h>
#include <math.h>

static const uint8_t CAN_CS_PIN  = 10;
static const uint8_t CAN_INT_PIN = 2;
static const uint8_t MOTOR_ID    = 69;

static const int16_t MAX_ERPM       = 3000;
static const int16_t MAX_ERPM_PER_S = 4000;
static const float   POS_TOL_DEG    = 2.0f;
static const uint32_t MOVE_TIMEOUT_MS = 8000;

MCP2515 mcp2515(CAN_CS_PIN);
struct can_frame rxFrame;

static float g_posDeg = NAN;
static float g_spdErpm = 0.0f;
static int8_t g_tempC = 0;
static uint8_t g_err = 0;
static bool g_haveStatus = false;

static uint32_t servoExtId(uint8_t functionId) {
  return ((uint32_t)functionId << 8) | MOTOR_ID;
}

static float wrap360(float deg) {
  float x = fmodf(deg, 360.0f);
  return (x < 0.0f) ? x + 360.0f : x;
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

static void appendI32BE(uint8_t *buf, int32_t v) {
  buf[0] = (uint8_t)(v >> 24);
  buf[1] = (uint8_t)(v >> 16);
  buf[2] = (uint8_t)(v >> 8);
  buf[3] = (uint8_t)v;
}

static void appendI16BE(uint8_t *buf, int idx, int16_t v) {
  buf[idx]     = (uint8_t)(v >> 8);
  buf[idx + 1] = (uint8_t)v;
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

static void parseStatusFrame() {
  int16_t posRaw = (int16_t)((rxFrame.data[0] << 8) | rxFrame.data[1]);
  int16_t spdRaw = (int16_t)((rxFrame.data[2] << 8) | rxFrame.data[3]);
  int16_t curRaw = (int16_t)((rxFrame.data[4] << 8) | rxFrame.data[5]);

  g_posDeg = posRaw * 0.1f;
  g_spdErpm = spdRaw * 10.0f;
  g_tempC = (int8_t)rxFrame.data[6];
  g_err = rxFrame.data[7];
  g_haveStatus = true;

  (void)curRaw;
}

static void printStatus() {
  if (!g_haveStatus) {
    Serial.println("No 0x29 status yet");
    return;
  }
  Serial.print("pos=");
  Serial.print(g_posDeg, 1);
  Serial.print(" deg (wrap ");
  Serial.print(wrap360(g_posDeg), 1);
  Serial.print(")  erpm=");
  Serial.print(g_spdErpm, 0);
  Serial.print("  temp=");
  Serial.print(g_tempC);
  Serial.print(" C  err=");
  Serial.println(g_err);
}

static void pollIncoming() {
  while (readCanFrame(0)) {
    const bool ext = (rxFrame.can_id & CAN_EFF_FLAG) != 0;
    if (!ext) {
      continue;
    }
    const uint32_t id = rxFrame.can_id & CAN_EFF_MASK;
    const uint8_t func = (uint8_t)(id >> 8);
    const uint8_t node = (uint8_t)(id & 0xFF);
    if (node != MOTOR_ID) {
      continue;
    }
    if (func == 0x2C) {
      const uint8_t reply[8] = {0xFA, 0xFB, 0xFC, 0xFD, 0, 0, 0, 0};
      sendExt(servoExtId(0x2C), reply, 8);
    } else if (func == 0x29 && rxFrame.can_dlc == 8) {
      parseStatusFrame();
    }
  }
}

static bool waitForStatus(uint32_t timeoutMs) {
  uint32_t end = millis() + timeoutMs;
  while (millis() < end) {
    pollIncoming();
    if (g_haveStatus) {
      return true;
    }
    delay(5);
  }
  return false;
}

static bool isHandshakeReply(const struct can_frame &f) {
  return f.can_dlc >= 4
      && f.data[0] == 0xFA
      && f.data[1] == 0xFB
      && f.data[2] == 0xFC
      && f.data[3] == 0xFD;
}

static bool processServoFrame(bool quiet) {
  const bool ext = (rxFrame.can_id & CAN_EFF_FLAG) != 0;
  if (!ext) {
    return false;
  }

  const uint32_t id = rxFrame.can_id & CAN_EFF_MASK;
  const uint8_t func = (uint8_t)(id >> 8);
  const uint8_t node = (uint8_t)(id & 0xFF);
  if (node != MOTOR_ID) {
    return false;
  }

  if (func == 0x2C) {
    const uint8_t reply[8] = {0xFA, 0xFB, 0xFC, 0xFD, 0, 0, 0, 0};
    sendExt(servoExtId(0x2C), reply, 8);
    if (!quiet) {
      Serial.println("Motor sent 0x2C, replied FA FB FC FD");
    }
    return true;
  }

  if (func == 0x29 && rxFrame.can_dlc == 8) {
    parseStatusFrame();
    if (!quiet) {
      Serial.println("Receiving 0x29 status (servo mode active)");
    }
    return true;
  }

  return isHandshakeReply(rxFrame);
}

static bool servoHandshake() {
  Serial.println("Connecting to motor...");

  // Motor may already stream 0x29 without a fresh FA FB FC FD reply.
  uint32_t listenEnd = millis() + 800;
  while (millis() < listenEnd) {
    if (readCanFrame(25) && processServoFrame(true)) {
      Serial.println("OK: motor already on CAN");
      printStatus();
      return true;
    }
  }

  const uint8_t zeros[8] = {0};
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    Serial.print("Handshake attempt ");
    Serial.println(attempt + 1);

    if (!sendExt(servoExtId(0x2C), zeros, 8)) {
      Serial.println("CAN TX failed");
      return false;
    }

    uint32_t deadline = millis() + 1500;
    while (millis() < deadline) {
      if (!readCanFrame(50)) {
        continue;
      }
      if (isHandshakeReply(rxFrame)) {
        Serial.println("OK: FA FB FC FD");
        return true;
      }
      if (processServoFrame(false)) {
        return true;
      }
    }
    delay(100);
  }

  return g_haveStatus;
}

static bool servoSetRpm(int32_t erpm) {
  uint8_t payload[4];
  appendI32BE(payload, erpm);
  return sendExt(servoExtId(3), payload, 4);
}

static bool servoSetPosSpd(float deg, int16_t erpm, int16_t erpmPerSec) {
  uint8_t buf[8];
  appendI32BE(buf, (int32_t)(deg * 10000.0f));
  appendI16BE(buf, 4, (int16_t)(erpm / 10));
  appendI16BE(buf, 6, (int16_t)(erpmPerSec / 10));
  return sendExt(servoExtId(6), buf, 8);
}

static bool servoSetOriginHere() {
  const uint8_t mode = 0;  // 0 = temporary origin (lost on power cycle)
  return sendExt(servoExtId(5), &mode, 1);
}

static void stopMotor() {
  servoSetRpm(0);
  Serial.println("Stop (RPM=0)");
}

static bool waitUntilPos(float targetDeg) {
  uint32_t end = millis() + MOVE_TIMEOUT_MS;
  while (millis() < end) {
    pollIncoming();
    if (g_haveStatus && fabsf(g_posDeg - targetDeg) <= POS_TOL_DEG
        && fabsf(g_spdErpm) < 100.0f) {
      printStatus();
      return true;
    }
    delay(10);
  }
  Serial.println("Move timeout");
  printStatus();
  return false;
}

static bool moveToAbsolute(float deg) {
  Serial.print("Move to ");
  Serial.print(deg, 1);
  Serial.println(" deg (absolute)");
  if (!servoSetPosSpd(deg, MAX_ERPM, MAX_ERPM_PER_S)) {
    Serial.println("CAN TX failed");
    return false;
  }
  return waitUntilPos(deg);
}

static bool moveRelative(float deltaDeg) {
  if (!g_haveStatus && !waitForStatus(2000)) {
    Serial.println("Need position from 0x29 frames first");
    return false;
  }
  return moveToAbsolute(g_posDeg + deltaDeg);
}

static void runDemo() {
  if (!waitForStatus(2000)) {
    Serial.println("No status frames — demo skipped");
    return;
  }

  Serial.println("Current position:");
  printStatus();

  const float start = g_posDeg;
  Serial.println("Demo: +30 -> +30 -> back to start (relative moves)");
  moveRelative(30.0f);
  moveRelative(30.0f);
  moveToAbsolute(start);
  stopMotor();
  Serial.println("Demo done.");
  Serial.println("Commands: p <abs deg> | r <delta deg> | z=zero here | s | h");
}

static void initCan() {
  pinMode(CAN_INT_PIN, INPUT);
  SPI.begin();
  mcp2515.reset();

  if (mcp2515.setBitrate(CAN_1000KBPS, MCP_16MHZ) != MCP2515::ERROR_OK
      || mcp2515.setNormalMode() != MCP2515::ERROR_OK) {
    Serial.println("MCP2515 init failed");
    while (true) {
      delay(1000);
    }
  }
}

static void handleSerial() {
  if (!Serial.available()) {
    return;
  }

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) {
    return;
  }

  if (line == "s") {
    stopMotor();
    return;
  }

  if (line == "h") {
    Serial.println(servoHandshake() ? "Handshake OK" : "Handshake failed");
    return;
  }

  if (line == "z") {
    if (servoSetOriginHere()) {
      Serial.println("Origin set at current position");
      g_posDeg = 0.0f;
    } else {
      Serial.println("Set origin failed");
    }
    return;
  }

  if (line.startsWith("p")) {
    moveToAbsolute(line.substring(1).toFloat());
    return;
  }

  if (line.startsWith("r")) {
    moveRelative(line.substring(1).toFloat());
    return;
  }

  Serial.println("Unknown. Use: p <abs> | r <delta> | z | s | h");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("=== AK40-10 CAN run ===");
  initCan();

  if (!servoHandshake()) {
    Serial.println("No handshake yet — waiting for 0x29 status...");
    if (!waitForStatus(3000)) {
      Serial.println("No motor data. Check power/CAN, then press reset or type h");
    }
  }

  delay(300);
  runDemo();
}

void loop() {
  pollIncoming();
  handleSerial();
}
