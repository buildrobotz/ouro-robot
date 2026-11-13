// Ouro, USB dongle. A second ESP32 DevKit plugged into the student's computer.
// Serial (921600 baud) <-> ESP-NOW. Frames on the wire: 0xAA 0x55, length, payload, XOR of payload.
// Board: "ESP32 Dev Module" (FQBN esp32:esp32:esp32), Arduino-ESP32 core 3.x.
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include "ouro_common.h"

const uint32_t BAUD = 921600;      // CP2102/CP2104/CH340C all do this; if frames drop, use 460800 on BOTH sides
const uint8_t PIN_LED = 2;          // toggles every 50 telemetry packets: steady flicker = robot heard
uint8_t broadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
QueueHandle_t telQ;                 // receive callback (Wi-Fi task) -> loop(): no Serial calls in the callback

void sendFrame(const uint8_t *p, uint8_t len) {
  uint8_t x = 0;  for (int i = 0; i < len; i++) x ^= p[i];
  uint8_t head[3] = {0xAA, 0x55, len};
  Serial.write(head, 3);  Serial.write(p, len);  Serial.write(x);
}

void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != (int)sizeof(TelPacket) || data[0] != TEL_MAGIC || data[1] != ROBOT_ID) return;
  TelPacket t;  memcpy(&t, data, sizeof(t));
  xQueueSend(telQ, &t, 0);          // never wait inside the Wi-Fi task; a full queue drops the packet
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  telQ = xQueueCreate(16, sizeof(TelPacket));
  Serial.setRxBufferSize(1024);
  Serial.setTxBufferSize(1024);
  Serial.begin(BAUD);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  bool ok = esp_now_init() == ESP_OK && esp_now_register_recv_cb(onRecv) == ESP_OK;
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, broadcastMac, 6);  peer.channel = WIFI_CHANNEL;  peer.ifidx = WIFI_IF_STA;
  ok = ok && esp_now_add_peer(&peer) == ESP_OK;
  if (!ok) while (true) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); delay(100); }   // fast blink = radio failed
}

void loop() {
  static uint32_t rxCount = 0;
  TelPacket t;
  while (xQueueReceive(telQ, &t, 0) == pdTRUE) {
    sendFrame((const uint8_t *)&t, sizeof(t));
    if (++rxCount % 50 == 0) digitalWrite(PIN_LED, !digitalRead(PIN_LED));
  }

  // Parse serial frames from the PC, forward valid commands as ESP-NOW broadcasts.
  static uint8_t buf[64], n = 0, need = 0, state = 0;
  while (Serial.available()) {
    uint8_t c = Serial.read();
    if (state == 0) { if (c == 0xAA) state = 1; }
    else if (state == 1) { state = (c == 0x55) ? 2 : (c == 0xAA ? 1 : 0); }
    else if (state == 2) { need = c; n = 0; state = (need > 0 && need <= sizeof(buf)) ? 3 : 0; }
    else if (state == 3) { buf[n++] = c; if (n == need) state = 4; }
    else {
      uint8_t x = 0;  for (int i = 0; i < need; i++) x ^= buf[i];
      if (x == c && need == sizeof(CmdPacket) && buf[0] == CMD_MAGIC && buf[1] == ROBOT_ID)
        esp_now_send(broadcastMac, buf, need);
      state = 0;
    }
  }
  vTaskDelay(1);                    // 1 ms: commands arrive every 20 ms, telemetry every 10 ms
}
