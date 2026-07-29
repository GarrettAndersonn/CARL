#include <Arduino.h>
#include <STM32_CAN.h>

#include "carl_protocol.h"
#include "messages.h"

using namespace carl;

STM32_CAN Can1(CAN1, ALT_2);  // PD0 RX, PD1 TX

int16_t teleopVx = 0;
int16_t teleopWz = 0;
bool estopLatched = false;

int testIdx = -1;
int16_t testVal = 0;

ThrottleStatus lastStatus = {TST_INIT, TF_NONE, 0, 0};
EncoderFb lastEnc = {0, 0};
bool haveStatus = false;

uint32_t lastCmdMs = 0;
uint32_t lastHbMs = 0;
uint32_t lastTlmMs = 0;
uint32_t lastTeleopMs = 0;
uint8_t hbCounter = 0;

char lineBuf[64];
uint8_t lineLen = 0;

// Jetson companion-link frame format:
// [0xAA][MSG_ID low][MSG_ID high][LEN][PAYLOAD][CRC8]
static constexpr uint8_t COMPANION_SOF = 0xAA;
static constexpr uint8_t COMPANION_MAX_PAYLOAD = 8;

static uint8_t companionCrc8(const uint8_t* data, size_t length) {
  uint8_t crc = 0;

  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];

    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 0x80) {
        crc = (uint8_t)(((crc << 1) ^ 0x07) & 0xFF);
      } else {
        crc = (uint8_t)((crc << 1) & 0xFF);
      }
    }
  }

  return crc;
}

static void sendCompanionFrame(
    uint16_t messageId,
    const uint8_t* payload,
    uint8_t payloadLength) {

  if (payloadLength > COMPANION_MAX_PAYLOAD) {
    return;
  }

  uint8_t frame[13];

  frame[0] = COMPANION_SOF;
  frame[1] = (uint8_t)(messageId & 0xFF);
  frame[2] = (uint8_t)((messageId >> 8) & 0xFF);
  frame[3] = payloadLength;

  for (uint8_t i = 0; i < payloadLength; ++i) {
    frame[4 + i] = payload[i];
  }

  const uint8_t bodyLength = 4 + payloadLength;
  frame[bodyLength] = companionCrc8(frame, bodyLength);

  Serial.write(frame, bodyLength + 1);
}

static int16_t clampMille(int32_t v) {
  if (v > THROTTLE_MAX) return THROTTLE_MAX;
  if (v < THROTTLE_MIN) return THROTTLE_MIN;
  return (int16_t)v;
}

static void canSend(uint16_t id, const uint8_t* buf, uint8_t len) {
  CAN_message_t msg;
  msg.id = id;
  msg.len = len;

  for (uint8_t i = 0; i < len && i < 8; i++) {
    msg.buf[i] = buf[i];
  }

  Can1.write(msg);
}

static void sendEstopFrame() {
  uint8_t buf[8];
  uint8_t len = pack_estop(buf, ESTOP_OPERATOR);
  canSend(CANID_ESTOP, buf, len);
}

static void sendHeartbeat(bool teleopOk) {
  Heartbeat h;
  h.counter = hbCounter++;
  h.flags = HB_FLAG_OK;

  if (teleopOk) h.flags |= HB_FLAG_TELEOP_OK;
  if (estopLatched) h.flags |= HB_FLAG_ESTOP;

  if (estopLatched) {
    h.mode = MODE_ESTOP;
  } else if (teleopVx != 0 || teleopWz != 0 || testIdx >= 0) {
    h.mode = MODE_TELEOP;
  } else {
    h.mode = MODE_IDLE;
  }

  uint8_t buf[8];
  uint8_t len = pack_heartbeat(buf, h);

  // Existing CAN heartbeat to the Teensy.
  canSend(CANID_HEARTBEAT, buf, len);

  // Binary heartbeat to the Jetson ROS 2 bridge.
  sendCompanionFrame(CANID_HEARTBEAT, buf, len);
}

static void sendThrottleCmd() {
  ThrottleCmd c = {0, 0, 0, 0};

  if (!estopLatched) {
    if (testIdx >= 0 && testIdx < 4) {
      int16_t v[4] = {0, 0, 0, 0};
      v[testIdx] = clampMille(testVal);
  uint8_t buf[8];
  uint8_t len = pack_throttle_cmd(buf, c);
  canSend(CANID_THROTTLE_CMD, buf, len);

  Serial.print("TX CMD: ");
// Existing CAN heartbeat to the Teensy.
canSend(CANID_HEARTBEAT, buf, len);

// New binary heartbeat to the Jetson ROS 2 bridge.
      c.fl = v[0];
      c.fr = v[1];
      c.rl = v[2];
      c.rr = v[3];
    } else {
      int16_t left  = clampMille((int32_t)teleopVx + teleopWz);
      int16_t right = clampMille((int32_t)teleopVx - teleopWz);

      c.fl = left;
      c.rl = left;
      c.fr = right;
      c.rr = right;
    }
  }

  uint8_t buf[8];
  uint8_t len = pack_throttle_cmd(buf, c);
  canSend(CANID_THROTTLE_CMD, buf, len);
  Serial.print("TX CMD: ");

Serial.print(c.fl);
Serial.print(" ");

Serial.print(c.fr);
Serial.print(" ");

Serial.print(c.rl);
Serial.print(" ");

Serial.println(c.rr);
}

static void parseLine(char* line) {
  if (line[0] == 'D' || line[0] == 'd') {
    int vx, wz;
    if (sscanf(line + 1, "%d %d", &vx, &wz) == 2) {
      teleopVx = clampMille(vx);
      teleopWz = clampMille(wz);
      testIdx = -1;
      lastTeleopMs = millis();
    }
  }

  else if (line[0] == 'M' || line[0] == 'm') {
    int idx, val;
    if (sscanf(line + 1, "%d %d", &idx, &val) == 2) {
      testIdx = idx;
      testVal = clampMille(val);
      teleopVx = 0;
      teleopWz = 0;
      lastTeleopMs = millis();
    }
  }

  else if (line[0] == 'S' || line[0] == 's' || line[0] == 'X' || line[0] == 'x') {
    teleopVx = 0;
    teleopWz = 0;
    testIdx = -1;
    testVal = 0;
    lastTeleopMs = millis();
  }

  else if (line[0] == 'E' || line[0] == 'e') {
    estopLatched = true;
    teleopVx = 0;
    teleopWz = 0;
    testIdx = -1;
    testVal = 0;
    sendEstopFrame();
    lastTeleopMs = millis();
  }

  else if (line[0] == 'R' || line[0] == 'r') {
    estopLatched = false;
    lastTeleopMs = millis();
  }
}

static void pollSerial() {
  while (Serial.available()) {
    char ch = (char)Serial.read();

    if (ch == '\n' || ch == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        parseLine(lineBuf);
        lineLen = 0;
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = ch;
    } else {
      lineLen = 0;
    }
  }
}

static void pollCan() {
  CAN_message_t rx;

  while (Can1.read(rx)) {
    switch (rx.id) {
      case CANID_THROTTLE_STATUS:
        if (unpack_throttle_status(rx.buf, rx.len, &lastStatus)) {
          haveStatus = true;
        }
        break;

      case CANID_ENCODER_FB:
        unpack_encoder_fb(rx.buf, rx.len, &lastEnc);
        break;

      case CANID_ESTOP:
        estopLatched = true;
        break;

      default:
        break;
    }
  }
}

static void streamTelemetry(bool teleopOk) {
  Serial.print(F("TLM hb="));
  Serial.print(hbCounter);

  Serial.print(F(" mode="));
  if (estopLatched) Serial.print(F("ESTOP"));
  else if (testIdx >= 0) Serial.print(F("TEST"));
  else if (teleopVx != 0 || teleopWz != 0) Serial.print(F("TELEOP"));
  else Serial.print(F("IDLE"));

  Serial.print(F(" test="));
  Serial.print(testIdx);

  Serial.print(F(" link="));
  Serial.print(teleopOk ? 1 : 0);

  Serial.print(F(" vx="));
  Serial.print(teleopVx);

  Serial.print(F(" wz="));
  Serial.print(teleopWz);

  Serial.print(F(" tstate="));
  Serial.print(haveStatus ? lastStatus.state : 255);

  Serial.print(F(" tflt="));
  Serial.print(haveStatus ? lastStatus.faults : 0);

  Serial.print(F(" L="));
  Serial.print(haveStatus ? lastStatus.applied_left : 0);

  Serial.print(F(" R="));
  Serial.print(haveStatus ? lastStatus.applied_right : 0);

  Serial.print(F(" encL="));
  Serial.print(lastEnc.left_count);

  Serial.print(F(" encR="));
  Serial.println(lastEnc.right_count);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Can1.begin();
  Can1.setBaudRate(CAN_BITRATE);

  lastTeleopMs = millis();

  Serial.println(F("AVL_CARL master/VCU online"));
}

void loop() {
  uint32_t now = millis();

  pollSerial();
  pollCan();

  bool teleopOk = (now - lastTeleopMs) <= TELEOP_TIMEOUT_MS;

  if (!teleopOk) {
    teleopVx = 0;
    teleopWz = 0;
    testIdx = -1;
    testVal = 0;
  }

  if (now - lastCmdMs >= THROTTLE_CMD_PERIOD_MS) {
    lastCmdMs = now;
    sendThrottleCmd();
  }

  if (now - lastHbMs >= HEARTBEAT_PERIOD_MS) {
    lastHbMs = now;
    sendHeartbeat(teleopOk);
  }

  if (now - lastTlmMs >= 100) {
    lastTlmMs = now;
    streamTelemetry(teleopOk);
  }
}

