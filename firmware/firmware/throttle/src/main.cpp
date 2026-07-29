#include <Arduino.h>
#include <Encoder.h>
#include <FlexCAN_T4.h>

#include "carl_protocol.h"
#include "heartbeat.h"
#include "messages.h"

using namespace carl;

struct MotorPins {
  uint8_t rpwm;
  uint8_t lpwm;
  uint8_t en;
};

// Your verified wiring
static const MotorPins kMotor[4] = {
  {2, 3, 10},   // FL
  {4, 5, 11},   // FR
  {8, 9, 24},   // RL
  {6, 7, 12},   // RR
};

// Your verified direction correction:
// FL normal, FR inverted, RL normal, RR inverted
static const int8_t kMotorDir[4] = {
  +1, -1, +1, -1
};

static const uint8_t ENC_RL_A = 14;
static const uint8_t ENC_RL_B = 15;
static const uint8_t ENC_RR_A = 16;
static const uint8_t ENC_RR_B = 17;

static const uint8_t PIN_LED = LED_BUILTIN;

static const uint32_t PWM_FREQ_HZ = 20000;
static const uint8_t PWM_RES_BITS = 12;
static const uint16_t PWM_MAX = 4095;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

Encoder encRL(ENC_RL_A, ENC_RL_B);
Encoder encRR(ENC_RR_A, ENC_RR_B);

Watchdog hbWatch(HEARTBEAT_TIMEOUT_MS);

ThrottleCmd cmd = {0, 0, 0, 0};

bool estopLatched = false;
bool haveHb = false;
uint8_t lastHbCounter = 0;

uint32_t lastStatusMs = 0;
uint32_t lastEncoderMs = 0;
uint32_t lastBlinkMs = 0;
uint32_t lastDebugMs = 0;

uint32_t rxCount = 0;
uint32_t hbRxCount = 0;
uint32_t cmdRxCount = 0;

static int16_t clampMille(int32_t v) {
  if (v > THROTTLE_MAX) return THROTTLE_MAX;
  if (v < THROTTLE_MIN) return THROTTLE_MIN;
  return (int16_t)v;
}

static uint16_t absMille(int16_t v) {
  return (v < 0) ? (uint16_t)(-v) : (uint16_t)v;
}

static void applyMotor(const MotorPins& m, int16_t mille, bool armed) {
  if (!armed) {
    analogWrite(m.rpwm, 0);
    analogWrite(m.lpwm, 0);
    digitalWrite(m.en, LOW);
    return;
  }

  mille = clampMille(mille);

  if (absMille(mille) < THROTTLE_DEADBAND) {
    analogWrite(m.rpwm, 0);
    analogWrite(m.lpwm, 0);
    digitalWrite(m.en, HIGH);
    return;
  }

  uint16_t duty = (uint16_t)((uint32_t)absMille(mille) * PWM_MAX / THROTTLE_MAX);

  digitalWrite(m.en, HIGH);

  if (mille > 0) {
    analogWrite(m.rpwm, duty);
    analogWrite(m.lpwm, 0);
  } else {
    analogWrite(m.rpwm, 0);
    analogWrite(m.lpwm, duty);
  }
}

static void driveAll(const ThrottleCmd& c, bool armed) {
  int16_t v[4] = {
    c.fl,
    c.fr,
    c.rl,
    c.rr
  };

  for (uint8_t i = 0; i < 4; i++) {
    applyMotor(kMotor[i], (int16_t)(v[i] * kMotorDir[i]), armed);
  }
}

static void coastAll() {
  ThrottleCmd zero = {0, 0, 0, 0};
  driveAll(zero, false);
}

static void canSend(uint16_t id, const uint8_t* buf, uint8_t len) {
  CAN_message_t msg;
  msg.id = id;
  msg.len = len;

  for (uint8_t i = 0; i < len && i < 8; i++) {
    msg.buf[i] = buf[i];
  }

  can1.write(msg);
}

static void sendStatus(uint8_t state, uint8_t faults) {
  ThrottleStatus s;

  s.state = state;
  s.faults = faults;
  s.applied_left = (int16_t)((cmd.fl + cmd.rl) / 2);
  s.applied_right = (int16_t)((cmd.fr + cmd.rr) / 2);

  uint8_t buf[8];
  uint8_t len = pack_throttle_status(buf, s);

  canSend(CANID_THROTTLE_STATUS, buf, len);
}

static void sendEncoders() {
  EncoderFb e;

  e.left_count = (int32_t)encRL.read();
  e.right_count = -(int32_t)encRR.read();

  uint8_t buf[8];
  uint8_t len = pack_encoder_fb(buf, e);

  canSend(CANID_ENCODER_FB, buf, len);
}

static void handleFrame(const CAN_message_t& msg) {
  rxCount++;

  switch (msg.id) {
    case CANID_ESTOP:
      estopLatched = true;
      break;

    case CANID_HEARTBEAT: {
      Heartbeat hb;

      if (unpack_heartbeat(msg.buf, msg.len, &hb)) {
        hbWatch.feed(millis());
        lastHbCounter = hb.counter;
        haveHb = true;
        Serial.println("Heartbeat Received");
        hbRxCount++;

        estopLatched = (hb.flags & HB_FLAG_ESTOP) != 0;
      }

      break;
    }

    case CANID_THROTTLE_CMD: {
      ThrottleCmd c;

      if (unpack_throttle_cmd(msg.buf, msg.len, &c)) {
        cmd.fl = clampMille(c.fl);
        cmd.fr = clampMille(c.fr);
        cmd.rl = clampMille(c.rl);
        cmd.rr = clampMille(c.rr);
        cmdRxCount++;
      }

      break;
    }

    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(PIN_LED, OUTPUT);

  analogWriteResolution(PWM_RES_BITS);

  for (uint8_t i = 0; i < 4; i++) {
    pinMode(kMotor[i].en, OUTPUT);
    pinMode(kMotor[i].rpwm, OUTPUT);
    pinMode(kMotor[i].lpwm, OUTPUT);

    digitalWrite(kMotor[i].en, LOW);

    analogWriteFrequency(kMotor[i].rpwm, PWM_FREQ_HZ);
    analogWriteFrequency(kMotor[i].lpwm, PWM_FREQ_HZ);
  }

  coastAll();

  can1.begin();
  can1.setBaudRate(CAN_BITRATE);
  can1.setMaxMB(16);
  can1.enableFIFO();

  Serial.println(F("THROTTLE FW BUILD 2026-07-22"));
  Serial.println(F("Waiting for heartbeat and throttle command..."));
}

void loop() {
  uint32_t now = millis();

  CAN_message_t rx;

  while (can1.read(rx)) {
    handleFrame(rx);
  }

  bool hbLost = hbWatch.expired(now);
  bool armed = haveHb && !hbLost && !estopLatched;

  uint8_t state;
  uint8_t faults = TF_NONE;

  if (estopLatched) {
    state = TST_ESTOP;
    faults |= TF_ESTOP;
  } else if (!haveHb) {
    state = TST_INIT;
  } else if (hbLost) {
    state = TST_SAFE_STOP;
    faults |= TF_HEARTBEAT_LOST;
  } else {
    bool moving =
      absMille(cmd.fl) >= THROTTLE_DEADBAND ||
      absMille(cmd.fr) >= THROTTLE_DEADBAND ||
      absMille(cmd.rl) >= THROTTLE_DEADBAND ||
      absMille(cmd.rr) >= THROTTLE_DEADBAND;

    state = moving ? TST_DRIVING : TST_IDLE;
  }

  if (armed) {
    driveAll(cmd, true);
  } else {
    coastAll();
  }

  if (now - lastEncoderMs >= ENCODER_FB_PERIOD_MS) {
    lastEncoderMs = now;
    sendEncoders();
  }

  if (now - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = now;
    sendStatus(state, faults);
  }

  if (armed) {
    digitalWrite(PIN_LED, HIGH);
  } else if (now - lastBlinkMs >= 100) {
    lastBlinkMs = now;
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
  }

  if (now - lastDebugMs >= 1000) {
    lastDebugMs = now;

    Serial.print(F("DBG rx="));
    Serial.print(rxCount);

    Serial.print(F(" hb="));
    Serial.print(hbRxCount);

    Serial.print(F(" cmd="));
    Serial.print(cmdRxCount);

    Serial.print(F(" haveHb="));
    Serial.print(haveHb ? 1 : 0);

    Serial.print(F(" hbLost="));
    Serial.print(hbLost ? 1 : 0);

    Serial.print(F(" estop="));
    Serial.print(estopLatched ? 1 : 0);

    Serial.print(F(" armed="));
    Serial.print(armed ? 1 : 0);

    Serial.print(F(" FL="));
    Serial.print(cmd.fl);

    Serial.print(F(" FR="));
    Serial.print(cmd.fr);

    Serial.print(F(" RL="));
    Serial.print(cmd.rl);

    Serial.print(F(" RR="));
    Serial.println(cmd.rr);
  }
}
