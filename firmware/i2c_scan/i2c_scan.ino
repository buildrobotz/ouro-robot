// Ouro bring-up: scan the I2C bus on GPIO21 (SDA) / GPIO22 (SCL) every 2 s and say what answered.
// Expected on a complete robot: 0x40 INA219, 0x68 mid MPU-6050, 0x69 head MPU-6050. Serial Monitor 115200.
// Board: "ESP32 Dev Module" (FQBN esp32:esp32:esp32).
#include <Wire.h>

const uint8_t PIN_SDA = 21, PIN_SCL = 22;

const char *name(uint8_t a) {
  switch (a) {
    case 0x40: return "INA219 (servo current)";
    case 0x68: return "MPU-6050 mid (AD0 low)";
    case 0x69: return "MPU-6050 head (AD0 high)";
    default:   return "unexpected";
  }
}

int scan(uint32_t hz) {
  Wire.setClock(hz);
  int found = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      found++;
      Serial.printf("  0x%02X  %s", a, name(a));
      if (a == 0x68 || a == 0x69) {                       // WHO_AM_I: 0x68 on a real MPU-6050
        Wire.beginTransmission(a); Wire.write(0x75); Wire.endTransmission(false);
        if (Wire.requestFrom(a, (size_t)1) == 1) Serial.printf("   WHO_AM_I 0x%02X", Wire.read());
      }
      if (a == 0x40) {                                    // config register: 0x399F after power-up
        Wire.beginTransmission(a); Wire.write(0x00); Wire.endTransmission(false);
        if (Wire.requestFrom(a, (size_t)2) == 2) { uint8_t h = Wire.read(), l = Wire.read(); Serial.printf("   config 0x%02X%02X", h, l); }
      }
      Serial.println();
    }
  }
  return found;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(PIN_SDA, INPUT);  pinMode(PIN_SCL, INPUT);
  Serial.printf("\ni2c_scan: idle levels SDA=%d SCL=%d (both must be 1: pull-ups present, nothing holding the bus)\n",
                digitalRead(PIN_SDA), digitalRead(PIN_SCL));
  Wire.begin(PIN_SDA, PIN_SCL, 100000);
  Wire.setTimeOut(5);
}

void loop() {
  int a = scan(100000);
  Serial.printf("100 kHz: %d device(s)\n", a);
  int b = scan(400000);
  Serial.printf("400 kHz: %d device(s)%s\n\n", b, a == b ? "" : "   <-- differs from 100 kHz: check wiring/pull-ups");
  delay(2000);
}
