// ─────────────────────────────────────────────────────────────────────────────
// AVL_CARL — STEERING node                       Board: Teensy 4.1 (ID 0x03)
//
// PHASE 1 STATUS: NOT USED. The kart turns by DIFFERENTIAL drive (left/right
// wheel speed split handled on the throttle node), so this node does not steer
// yet. The physical board is currently UNPOWERED; its CAN transceiver is left
// with the 120 Ω termination enabled as a bus end.
//
// This firmware is intentionally a SAFE-IDLE stub: if the board ever gets
// powered, it keeps the CL57T DISABLED, never emits a step pulse, listens to the
// master heartbeat, and reports a "homed=0 / idle" status. No motion can occur.
//
// When real Ackermann steering is implemented, replace the body of loop() with
// step/dir generation toward the commanded angle (CANID_STEERING_CMD 0x110) and
// publish CANID_STEERING_STATUS (0x220). Pin reservations are below.
//
// CAN: FlexCAN_T4 on CAN1 -> Teensy pin 22 = CAN1_TX, pin 23 = CAN1_RX.
// ─────────────────────────────────────────────────────────────────────────────
#include <Arduino.h>
#include <FlexCAN_T4.h>

#include "carl_protocol.h"
#include "heartbeat.h"
#include "messages.h"

using namespace carl;

// ── Pin reservations for the CL57T (future use) ─────────────────────────────
// Optocoupler inputs: PUL/step, DIR, ENA. Verify 3.3 V drive vs the driver's
// opto current (see docs/hardware.md §7) before wiring.
static const uint8_t PIN_STEP = 3;
static const uint8_t PIN_DIR = 4;
static const uint8_t PIN_ENA = 5;  // held in the DISABLED sense while idle
static const uint8_t PIN_LED = LED_BUILTIN;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
Watchdog hbWatch(HEARTBEAT_TIMEOUT_MS);

uint32_t lastStatusMs = 0;
uint32_t lastBlinkMs = 0;

// Keep the stepper electrically idle: no step pulses, driver disabled.
static void holdDisabled() {
  digitalWrite(PIN_STEP, LOW);
  digitalWrite(PIN_ENA, LOW);  // de-assert enable -> CL57T not driving
}

static void canSend(uint16_t id, const uint8_t* buf, uint8_t len) {
  CAN_message_t msg;
  msg.id = id;
  msg.len = len;
  for (uint8_t i = 0; i < len && i < 8; i++) msg.buf[i] = buf[i];
  can1.write(msg);
}

static void sendStatus() {
  // Minimal steering status: byte0 = state (0 = idle/not-implemented),
  // byte1 = homed flag (0), rest reserved. Mirrors the 0x220 slot.
  uint8_t buf[8] = {0};
  buf[0] = 0;  // idle
  buf[1] = 0;  // not homed
  canSend(CANID_STEERING_STATUS, buf, 2);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_ENA, OUTPUT);
  holdDisabled();

  can1.begin();
  can1.setBaudRate(CAN_BITRATE);
  can1.setMaxMB(16);
  can1.enableFIFO();

  Serial.println(F("AVL_CARL steering node (Teensy 4.1) — SAFE-IDLE (Phase 1, differential steering)"));
}

void loop() {
  const uint32_t now = millis();

  // Drain CAN so the controller never overflows; feed the heartbeat watchdog.
  CAN_message_t rx;
  while (can1.read(rx)) {
    if (rx.id == CANID_HEARTBEAT) hbWatch.feed(now);
    // CANID_STEERING_CMD is intentionally ignored in Phase 1.
  }

  // No matter what, this node holds the stepper disabled.
  holdDisabled();

  if (now - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = now;
    sendStatus();
  }

  // Slow heartbeat-style blink to show the node is alive but idle.
  if (now - lastBlinkMs >= 500) {
    lastBlinkMs = now;
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
  }
}
