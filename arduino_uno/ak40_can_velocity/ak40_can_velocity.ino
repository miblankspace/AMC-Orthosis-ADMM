/*
 * AK40-10 continuous velocity (servo CAN RPM mode)
 * Arduino Uno R4 + Joy-IT SBC-CAN01
 *
 * Library: "mcp2515" by autowp
 *
 * Serial @ 115200:
 *   v <erpm>   set speed (electrical RPM; negative = reverse)
 *   s          stop
 *   i          print status
 *   h          reconnect
 *
 * Change DEFAULT_ERPM below or send v <value> after upload.
 * Set AUTO_START false to wait for a v command.
 */

#include <SPI.h>
#include <mcp2515.h>

static const uint8_t CAN_CS_PIN  = 10;
static const uint8_t CAN_INT_PIN = 2;
static const uint8_t MOTOR_ID    = 69;

// --- tune here ---
static const int32_t DEFAULT_ERPM = 800;   // start speed (+ = one direction)
static const int32_t MAX_ERPM     = 3000;  // safety clamp
static const bool    AUTO_START   = true;  // spin on boot after connect
static const uint32_t CMD_PERIOD_MS = 100; // re-send RPM setpoint
static const uint32_t STATUS_PERIOD_MS = 500;

MCP2515 mcp2515(CAN_CS_PIN);
struct can_frame rxFrame;

static int32_t g_targetErpm = 0;
static float g_posDeg = 0.0f;
static float g_spdErpm = 0.0f;
static int8_t g_tempC = 0;
static uint8_t g_err = 0;
static bool g_haveStatus = false;
static uint32_t g_lastCmdMs = 0;
static uint32_t g_lastStatusPrintMs = 0;

static uint32_t servoExtId(uint8_t functionId) {
  return ((uint32_t)functionId << 8) | MOTOR_ID;
}

static int32_t clampErpm(int32_t erpm) {
  if (erpm > MAX_ERPM) return MAX_ERPM;
  if (erpm < -MAX_ERPM) return -MAX_ERPM;
  return erpm;
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
  g_posDeg = posRaw * 0.1f;
  g_spdErpm = spdRaw * 10.0f;
  g_tempC = (int8_t)rxFrame.data[6];
  g_err = rxFrame.data[7];
  g_haveStatus = true;
}

static void printStatus() {
  Serial.print("cmd_erpm=");
  Serial.print(g_targetErpm);
  Serial.print("  meas_erpm=");
  Serial.print(g_spdErpm, 0);
  Serial.print("  pos=");
  Serial.print(g_posDeg, 1);
  Serial.print(" deg  temp=");
  Serial.print(g_tempC);
  Serial.print(" C  err=");
  Serial.println(g_err);
}

static void pollIncoming() {
  while (readCanFrame(0)) {
    if ((rxFrame.can_id & CAN_EFF_FLAG) == 0) {
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

static bool isHandshakeReply(const struct can_frame &f) {
  return f.can_dlc >= 4
      && f.data[0] == 0xFA && f.data[1] == 0xFB
      && f.data[2] == 0xFC && f.data[3] == 0xFD;
}

static bool processServoFrame() {
  if ((rxFrame.can_id & CAN_EFF_FLAG) == 0) {
    return isHandshakeReply(rxFrame);
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
    return true;
  }
  if (func == 0x29 && rxFrame.can_dlc == 8) {
    parseStatusFrame();
    return true;
  }
  return isHandshakeReply(rxFrame);
}

static bool servoConnect() {
  Serial.println("Connecting...");
  uint32_t end = millis() + 800;
  while (millis() < end) {
    if (readCanFrame(25) && processServoFrame()) {
      Serial.println("OK: motor on CAN");
      return true;
    }
  }

  const uint8_t zeros[8] = {0};
  for (uint8_t n = 0; n < 3; n++) {
    sendExt(servoExtId(0x2C), zeros, 8);
    end = millis() + 1500;
    while (millis() < end) {
      if (!readCanFrame(50)) {
        continue;
      }
      if (isHandshakeReply(rxFrame) || processServoFrame()) {
        Serial.println("OK: connected");
        return true;
      }
    }
  }
  return g_haveStatus;
}

static bool servoSetRpm(int32_t erpm) {
  uint8_t payload[4];
  appendI32BE(payload, erpm);
  return sendExt(servoExtId(3), payload, 4);  // CAN_PACKET_SET_RPM
}

static void setSpeed(int32_t erpm) {
  g_targetErpm = clampErpm(erpm);
  if (servoSetRpm(g_targetErpm)) {
    Serial.print("Speed set to ");
    Serial.print(g_targetErpm);
    Serial.println(" ERPM");
  } else {
    Serial.println("CAN TX failed");
  }
  g_lastCmdMs = 0;  // send immediately in loop
}

static void stopMotor() {
  setSpeed(0);
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
  if (line == "i") {
    printStatus();
    return;
  }
  if (line == "h") {
    Serial.println(servoConnect() ? "Reconnected" : "Reconnect failed");
    return;
  }
  if (line.startsWith("v")) {
    setSpeed((int32_t)line.substring(1).toInt());
    return;
  }
  Serial.println("Use: v <erpm> | s | i | h");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== AK40-10 velocity mode ===");
  initCan();

  if (!servoConnect()) {
    Serial.println("No motor yet — type h after power-on");
  }

  Serial.print("Max |ERPM|=");
  Serial.println(MAX_ERPM);
  Serial.println("Commands: v <erpm> | s=stop | i=status | h=reconnect");

  if (AUTO_START) {
    setSpeed(DEFAULT_ERPM);
  } else {
    Serial.println("AUTO_START off — send e.g. v 800");
  }
}

void loop() {
  pollIncoming();
  handleSerial();

  if (g_targetErpm != 0 && millis() - g_lastCmdMs >= CMD_PERIOD_MS) {
    servoSetRpm(g_targetErpm);
    g_lastCmdMs = millis();
  }

  if (g_haveStatus && millis() - g_lastStatusPrintMs >= STATUS_PERIOD_MS) {
    printStatus();
    g_lastStatusPrintMs = millis();
  }

  if (g_err != 0) {
    Serial.print("FAULT ");
    Serial.println(g_err);
    stopMotor();
    delay(1000);
  }
}
