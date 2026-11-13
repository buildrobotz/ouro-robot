// Ouro assembly helper: centre one servo at 1500 us so the horn can be fitted with the link straight.
// Flash to the ROBOT's ESP32, open Serial Monitor at 115200, battery plugged in (servo power comes from the board).
//   1..8   pick the joint (uses the robot's pin for that joint, see docs/electronics.md)
//   c      centre: 1500 us          s   sweep 1000 -> 2000 -> 1500 us, slowly
//   + / -  nudge 10 us              l   limp (no pulses)
// Board: "ESP32 Dev Module" (FQBN esp32:esp32:esp32), library ESP32Servo 3.x.
#include <ESP32Servo.h>

const uint8_t SERVO_PIN[8] = {13, 26, 16, 17, 18, 19, 23, 25};   // same map as ouro_robot.ino
Servo sv;
int joint = 0, us = 1500;

void drive(int pulse) {
  us = constrain(pulse, 500, 2500);
  sv.writeMicroseconds(us);
  Serial.printf("joint %d (GPIO %d): %d us\n", joint + 1, SERVO_PIN[joint], us);
}

void pick(int j) {
  if (sv.attached()) { sv.detach(); }
  pinMode(SERVO_PIN[joint], OUTPUT);  digitalWrite(SERVO_PIN[joint], LOW);   // old pin: quiet
  joint = j;
  sv.setPeriodHertz(50);
  sv.attach(SERVO_PIN[joint], 500, 2500);
  sv.setTimerWidth(16);
  drive(1500);
}

void setup() {
  Serial.begin(115200);
  for (uint8_t p : SERVO_PIN) { pinMode(p, OUTPUT);  digitalWrite(p, LOW); }
  delay(300);
  Serial.println("\nservo_center: 1..8 joint, c centre, s sweep, +/- 10 us, l limp");
  pick(0);
}

void loop() {
  if (!Serial.available()) { delay(5); return; }
  char k = Serial.read();
  if (k >= '1' && k <= '8') pick(k - '1');
  else if (k == 'c') { if (!sv.attached()) pick(joint); drive(1500); }
  else if (k == '+') drive(us + 10);
  else if (k == '-') drive(us - 10);
  else if (k == 'l') { sv.release(); Serial.println("limp"); }
  else if (k == 's') {
    if (!sv.attached()) pick(joint);
    for (int p = us; p >= 1000; p -= 5) { sv.writeMicroseconds(p); delay(10); }
    for (int p = 1000; p <= 2000; p += 5) { sv.writeMicroseconds(p); delay(10); }
    for (int p = 2000; p >= 1500; p -= 5) { sv.writeMicroseconds(p); delay(10); }
    drive(1500);
  }
}
