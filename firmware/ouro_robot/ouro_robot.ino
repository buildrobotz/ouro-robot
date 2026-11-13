// Ouro, on-robot firmware. ESP32 DevKit (ESP32-WROOM-32, classic ESP32), Arduino-ESP32 core 3.x,
// library ESP32Servo 3.x. Board: "ESP32 Dev Module" (FQBN esp32:esp32:esp32).
// Job: move 8 servos to the pulse widths it is told, and report 2 IMUs + current + latch + battery.
// Nothing else: no control loop runs here. See docs/electronics.md for the wiring and bring-up.
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <esp_sleep.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include "ouro_common.h"

// ---------------------------------------------------------------- pins (docs/electronics.md, pin map)
// Joint 2 moved from GPIO14 to GPIO26: GPIO14 is a JTAG pin that is not quiet during boot.
const uint8_t SERVO_PIN[N_SERVO] = {13, 26, 16, 17, 18, 19, 23, 25};  // joint 1 (tail) .. joint 8 (neck)
const uint8_t PIN_SDA = 21, PIN_SCL = 22, PIN_LATCH = 27, PIN_BATT = 34, PIN_LED = 2;
const uint8_t MPU_MID = 0x68, MPU_HEAD = 0x69, INA = 0x40;

// ---------------------------------------------------------------- tuning
const uint32_t I2C_HZ = 400000;          // 300 mm bus, ~1-1.9 k pull-ups: rise time ~60-110 ns (limit 300 ns)
const uint16_t I2C_TIMEOUT_MS = 3;       // a stuck bus costs 3 ms, never the 50 ms default
const float    INA_MA_PER_LSB = 0.2f;    // 10 uV/LSB over 0.05 ohm (module's 0.1 + added 0.1 in parallel).
                                         // Without the added shunt use 0.1f. Trim this against a meter.
const uint16_t INA_CONFIG = 0x19FF;      // 16 V bus, PGA /8 (+-320 mV), bus 12-bit, shunt 128-sample average
                                         // (68 ms: spans ~3 servo frames, so 50 Hz current bursts average out)
const float    BATT_DIV = (100.0f + 33.0f) / 33.0f;   // 100k over 33k
const float    BATT_CAL = 1.000f;        // trim: meter reading / reported value (step 6 of bring-up)
const uint16_t LOW_BATT_MV = 6600;       // 3.3 V per cell under load
const uint32_t LOW_BATT_HOLD_MS = 2000;  // 2 s of sag, not one stall spike
const uint32_t SLEEP_AFTER_LOW_MS = 30000;   // keep reporting low battery for 30 s, then deep sleep
const uint16_t RAIL_LOW_MV = 4500;       // servo rail sagging: flag it
const uint32_t CMD_TIMEOUT_MS = 300;
const uint16_t MAX_STEP_US = 32;         // per 5 ms tick = 6400 us/s = 10 rad/s, the top of the sim's slew range
const uint32_t TICK_MS = 5;              // 200 Hz slew, 100 Hz telemetry

// ---------------------------------------------------------------- state
Servo servo[N_SERVO];
uint8_t broadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Written by the ESP-NOW receive callback (Wi-Fi task, core 0), read by loop() (core 1).
// The spinlock makes the 20-byte copy atomic: loop() never sees half of one command and half of the next.
portMUX_TYPE cmdMux = portMUX_INITIALIZER_UNLOCKED;
CmdPacket rxCmd = {};  uint32_t rxMs = 0;  bool rxAny = false;

uint16_t pulseNow[N_SERVO];
bool live = false;  uint8_t seq = 0;
float battFilt = 0;  uint32_t lowBattSince = 0, lowBattAt = 0;  bool lowBatt = false;
int16_t lastMid[6] = {}, lastHead[6] = {};  int16_t lastmA = 0;  uint16_t railmV = 0;

struct Dev { uint8_t addr; bool ok; uint8_t fails; };
Dev devMid = {MPU_MID, false, 0}, devHead = {MPU_HEAD, false, 0}, devIna = {INA, false, 0};

// ---------------------------------------------------------------- ESP-NOW
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != (int)sizeof(CmdPacket) || data[0] != CMD_MAGIC || data[1] != ROBOT_ID) return;
  portENTER_CRITICAL(&cmdMux);
  memcpy(&rxCmd, data, sizeof(CmdPacket));
  rxMs = millis();  rxAny = true;
  portEXIT_CRITICAL(&cmdMux);
}

// ---------------------------------------------------------------- I2C helpers (never block more than a few ms)
bool wr8(uint8_t dev, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(dev); Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}
bool wr16(uint8_t dev, uint8_t reg, uint16_t val) {
  Wire.beginTransmission(dev); Wire.write(reg); Wire.write((uint8_t)(val >> 8)); Wire.write((uint8_t)val);
  return Wire.endTransmission() == 0;
}
bool rd(uint8_t dev, uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(dev); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(dev, (size_t)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = (uint8_t)Wire.read();   // one statement per byte: defined order
  return true;
}
inline int16_t be16(const uint8_t *b) { return (int16_t)(((uint16_t)b[0] << 8) | b[1]); }

// A slave left holding SDA low (power glitch or a wire breaking mid-byte) hangs the bus until it sees
// clocks. Clock SCL up to 9 times, send a STOP, restart the peripheral.
void i2cRecover() {
  Wire.end();
  pinMode(PIN_SDA, INPUT_PULLUP);
  pinMode(PIN_SCL, OUTPUT_OPEN_DRAIN);  digitalWrite(PIN_SCL, HIGH);
  for (int i = 0; i < 9 && digitalRead(PIN_SDA) == LOW; i++) {
    digitalWrite(PIN_SCL, LOW);  delayMicroseconds(5);  digitalWrite(PIN_SCL, HIGH);  delayMicroseconds(5);
  }
  pinMode(PIN_SDA, OUTPUT_OPEN_DRAIN);
  digitalWrite(PIN_SDA, LOW);  delayMicroseconds(5);  digitalWrite(PIN_SCL, HIGH);  delayMicroseconds(5);
  digitalWrite(PIN_SDA, HIGH);  delayMicroseconds(5);
  Wire.begin(PIN_SDA, PIN_SCL, I2C_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);
}

bool initMpu(Dev &d) {
  d.ok = wr8(d.addr, 0x6B, 0x01)      // wake, PLL on gyro X
      && wr8(d.addr, 0x1A, 0x03)      // DLPF 44 Hz: kills servo buzz
      && wr8(d.addr, 0x1B, 0x10)      // +-1000 dps
      && wr8(d.addr, 0x1C, 0x08);     // +-4 g
  d.fails = 0;
  return d.ok;
}
bool initIna(Dev &d) { d.ok = wr16(d.addr, 0x00, INA_CONFIG);  d.fails = 0;  return d.ok; }

void markFail(Dev &d) { if (d.ok && ++d.fails >= 3) d.ok = false; }

bool readMpu(Dev &d, int16_t *out) {            // accel xyz, (temp skipped), gyro xyz
  if (!d.ok) return false;
  uint8_t b[14];
  if (!rd(d.addr, 0x3B, b, 14)) { markFail(d); return false; }
  d.fails = 0;
  out[0] = be16(b + 0);  out[1] = be16(b + 2);  out[2] = be16(b + 4);
  out[3] = be16(b + 8);  out[4] = be16(b + 10); out[5] = be16(b + 12);
  return true;
}

bool readIna(Dev &d) {                          // shunt (signed, 10 uV/LSB) and bus voltage (4 mV/LSB)
  if (!d.ok) return false;
  uint8_t b[2];
  if (!rd(d.addr, 0x01, b, 2)) { markFail(d); return false; }
  int32_t mA = lroundf(be16(b) * INA_MA_PER_LSB);
  lastmA = (int16_t)constrain(mA, -32000, 32000);
  if (!rd(d.addr, 0x02, b, 2)) { markFail(d); return false; }
  railmV = (uint16_t)((((uint16_t)b[0] << 8) | b[1]) >> 3) * 4;
  d.fails = 0;
  return true;
}

// ---------------------------------------------------------------- servos
// Channels are allocated once at boot and never freed. "Limp" = duty 0 (pin held low, no pulses),
// which an analog hobby servo treats exactly like an unplugged signal wire.
void setLive(bool on) {
  if (on == live) return;
  for (int i = 0; i < N_SERVO; i++) {
    if (on) servo[i].writeMicroseconds(pulseNow[i]);
    else    servo[i].release();
  }
  live = on;
}

void goToSleep() {
  setLive(false);
  wr8(MPU_MID, 0x6B, 0x40);  wr8(MPU_HEAD, 0x6B, 0x40);   // IMUs to sleep
  wr16(INA, 0x00, INA_CONFIG & ~0x0007);                    // INA219 power-down
  digitalWrite(PIN_LED, LOW);
  esp_now_deinit();  WiFi.mode(WIFI_OFF);
  Serial.println("low battery: deep sleep. UNPLUG THE BATTERY NOW.");  Serial.flush();
  esp_deep_sleep_start();                                    // no wake source: only reset or replug
}

// ---------------------------------------------------------------- setup
void setup() {
  Serial.begin(115200);
  // Drive every servo signal low before anything else: a floating 30 cm servo lead twitches the servo.
  for (int i = 0; i < N_SERVO; i++) { pinMode(SERVO_PIN[i], OUTPUT);  digitalWrite(SERVO_PIN[i], LOW); }
  pinMode(PIN_LED, OUTPUT);  pinMode(PIN_LATCH, INPUT_PULLUP);
  analogSetPinAttenuation(PIN_BATT, ADC_11db);

  for (int i = 0; i < N_SERVO; i++) {
    pulseNow[i] = 1500;
    servo[i].setPeriodHertz(50);
    servo[i].attach(SERVO_PIN[i], 500, 2500);
    servo[i].setTimerWidth(16);      // attach() defaults to 10 bits = 19.5 us steps; 16 bits = 0.3 us
    servo[i].release();              // allocated, but no pulses until the first enabled command
  }

  Wire.begin(PIN_SDA, PIN_SCL, I2C_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);
  if (digitalRead(PIN_SDA) == LOW) i2cRecover();
  initMpu(devMid);  initMpu(devHead);  initIna(devIna);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                                   // radio always on: no 100 ms modem-sleep gaps
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  bool nowOk = esp_now_init() == ESP_OK;
  nowOk = nowOk && esp_now_register_recv_cb(onRecv) == ESP_OK;
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, broadcastMac, 6);  peer.channel = WIFI_CHANNEL;  peer.ifidx = WIFI_IF_STA;
  nowOk = nowOk && esp_now_add_peer(&peer) == ESP_OK;

  battFilt = analogReadMilliVolts(PIN_BATT) * BATT_DIV * BATT_CAL;
  Serial.printf("\nOuro robot  id %d  ch %d  MAC %s  esp-now %s\n", ROBOT_ID, WIFI_CHANNEL,
                WiFi.macAddress().c_str(), nowOk ? "ok" : "FAILED");
  Serial.printf("mid IMU 0x68 %s  head IMU 0x69 %s  INA219 0x40 %s  battery %.2f V\n",
                devMid.ok ? "ok" : "MISSING", devHead.ok ? "ok" : "MISSING", devIna.ok ? "ok" : "MISSING",
                battFilt / 1000.0f);
  if (!nowOk) while (true) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); delay(100); }   // fast blink forever
}

// ---------------------------------------------------------------- loop
void loop() {
  static uint32_t nextTick = 0, tick = 0, lastRetry = 0, lastPrint = 0;
  static uint8_t latchCount = 0;
  uint32_t now = millis();
  if ((int32_t)(now - nextTick) < 0) { vTaskDelay(1); return; }
  nextTick = now + TICK_MS;  tick++;

  // One consistent snapshot of the latest command.
  CmdPacket c;  uint32_t cmdMs;  bool any;
  portENTER_CRITICAL(&cmdMux);
  c = rxCmd;  cmdMs = rxMs;  any = rxAny;
  portEXIT_CRITICAL(&cmdMux);
  // Signed difference: the callback may stamp rxMs a millisecond after `now` was read.
  bool timeout = !any || (int32_t)(now - cmdMs) > (int32_t)CMD_TIMEOUT_MS;

  // Battery: ADC1 (works with Wi-Fi on), eFuse-calibrated mV, ~80 ms low-pass.
  float b = analogReadMilliVolts(PIN_BATT) * BATT_DIV * BATT_CAL;
  battFilt += (b - battFilt) / 16.0f;
  uint16_t batt = (uint16_t)battFilt;
  bool onBattery = batt > 3000;                          // under 3 V = no pack (USB-powered bench test)
  if (onBattery && batt < LOW_BATT_MV) { if (!lowBattSince) lowBattSince = now ? now : 1; } else lowBattSince = 0;
  if (!lowBatt && lowBattSince && now - lowBattSince > LOW_BATT_HOLD_MS) { lowBatt = true; lowBattAt = now; }
  if (lowBatt && now - lowBattAt > SLEEP_AFTER_LOW_MS) goToSleep();   // latched: only a reset clears it

  bool want = !timeout && !lowBatt && (c.flags & 1);
  if (want && !live) for (int i = 0; i < N_SERVO; i++) pulseNow[i] = constrain((int)c.pulse_us[i], 500, 2500);
  setLive(want);
  if (live) {
    for (int i = 0; i < N_SERVO; i++) {
      int target = constrain((int)c.pulse_us[i], 500, 2500);
      int d = constrain(target - (int)pulseNow[i], -(int)MAX_STEP_US, (int)MAX_STEP_US);
      pulseNow[i] += d;
      servo[i].writeMicroseconds(pulseNow[i]);
    }
  }

  // Latch sense: GPIO27 and GND go to the two steel head pins; a bare wire across the tail cradle joins them.
  latchCount = (digitalRead(PIN_LATCH) == LOW) ? min(latchCount + 1, 6) : max(latchCount - 1, 0);
  uint32_t blink = lowBatt ? (tick / 20) & 1 : live ? 1 : (tick % 200) < 10;   // idle: 50 ms heartbeat each second
  digitalWrite(PIN_LED, blink);

  if (tick & 1) return;                                  // telemetry every second tick = 100 Hz

  // Re-initialise a sensor that dropped out (re-plugged wire, power glitch), at most every 500 ms.
  if (now - lastRetry > 500 && (!devMid.ok || !devHead.ok || !devIna.ok)) {
    lastRetry = now;
    if (digitalRead(PIN_SDA) == LOW || digitalRead(PIN_SCL) == LOW) i2cRecover();
    if (!devMid.ok) initMpu(devMid);
    if (!devHead.ok) initMpu(devHead);
    if (!devIna.ok) initIna(devIna);
  }

  TelPacket t = {};
  t.magic = TEL_MAGIC;  t.robot_id = ROBOT_ID;  t.seq = seq++;  t.last_cmd_seq = c.seq;  t.t_ms = now;
  readMpu(devMid, lastMid);  readMpu(devHead, lastHead);  readIna(devIna);
  memcpy(t.mid, lastMid, sizeof(t.mid));  memcpy(t.head, lastHead, sizeof(t.head));
  t.servo_mA = lastmA;  t.batt_mV = batt;
  t.latch = latchCount >= 3;                             // 3 of the last ticks closed = 15 ms of contact
  t.status = (live ? ST_LIVE : 0) | (lowBatt ? ST_LOW_BATT : 0) | (timeout ? ST_TIMEOUT : 0)
           | (live && devIna.ok && railmV < RAIL_LOW_MV ? ST_RAIL_LOW : 0)
           | (devMid.ok ? 0 : ST_MID_FAULT) | (devHead.ok ? 0 : ST_HEAD_FAULT) | (devIna.ok ? 0 : ST_INA_FAULT);
  esp_now_send(broadcastMac, (const uint8_t *)&t, sizeof(t));   // broadcast: no ACK, no retry, ~0.9 ms airtime

  if (now - lastPrint > 1000) {                          // 1 Hz status line for bring-up over USB
    lastPrint = now;
    Serial.printf("batt %.2f V  rail %.2f V  %d mA  latch %d  status 0x%02X  az_mid %d  az_head %d\n",
                  batt / 1000.0f, railmV / 1000.0f, lastmA, t.latch, t.status, lastMid[2], lastHead[2]);
  }
}
