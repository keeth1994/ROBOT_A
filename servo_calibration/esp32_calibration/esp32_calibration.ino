// Library: Adafruit PWM Servo Driver Library. USB serial: 115200 baud.
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm(0x40, Wire);
constexpr uint8_t channels[] = {0, 1, 2, 3, 8, 9, 10, 11};
constexpr int MIN_US = 500, MAX_US = 2500; // Electrical bounds, not safe joint limits.
int active = -1, pulse = 1500;
bool ready = false;
uint32_t lastContact = 0;
char line[64];
size_t used = 0;
bool overflow = false;

bool off() {
  bool ok = true;
  for (auto channel : channels) if (pwm.setPWM(channel, 0, 4096) != 0) ok = false;
  active = -1;
  if (!ok) ready = false;
  return ok;
}
bool writePulse(int value) {
  if (pwm.setPWM(active, 0, (uint16_t)lround(value * 4096.0 * 50 / 1000000)) != 0) {
    off(); ready = false; Serial.println("ERR I2C fault; reset required"); return false;
  }
  pulse = value;
  Serial.printf("OK PULSE %d %d\n", active, pulse);
  return true;
}
void command(const char *s) {
  int value; char extra;
  if (!strcmp(s, "PING")) { lastContact = millis(); Serial.println("OK PING"); }
  else if (!strcmp(s, "OFF")) Serial.println(off() ? "OK OFF" : "ERR OFF I2C; cut servo power");
  else if (!strcmp(s, "STATUS")) Serial.printf("OK STATUS ready=%d channel=%d pulse=%d\n", ready, active, pulse);
  else if (sscanf(s, "SELECT %d %c", &value, &extra) == 1) {
    bool allowed = false;
    for (auto channel : channels) if (value == channel) allowed = true;
    if (!ready || !allowed) { Serial.println("ERR unavailable channel/controller"); return; }
    if (!off()) { Serial.println("ERR OFF I2C; cut servo power"); return; }
    active = value; lastContact = millis(); writePulse(1500);
  } else if (sscanf(s, "JOG %d %c", &value, &extra) == 1) {
    if (!ready || active < 0 || value < -10 || value > 10 ||
        pulse + value < MIN_US || pulse + value > MAX_US) {
      Serial.println("ERR jog requires selected channel, step <=10 us and pulse 500..2500"); return;
    }
    lastContact = millis(); writePulse(pulse + value);
  } else Serial.println("ERR expected STATUS SELECT channel JOG delta PING OFF");
}
void setup() {
  Serial.begin(115200); Wire.begin(21, 22);
  ready = pwm.begin();
  if (ready) { pwm.setPWMFreq(50); ready = off(); }
  Serial.println("CALIBRATION v1; outputs off; SELECT starts at 1500 us");
}
void loop() {
  if (active >= 0 && millis() - lastContact > 1500) {
    Serial.println(off() ? "TIMEOUT outputs off" : "ERR timeout I2C; cut servo power");
  }
  for (int n = 0; n < 64 && Serial.available(); ++n) {
    char c = Serial.read();
    if (c == '\n') {
      if (!overflow) { line[used] = 0; command(line); }
      else Serial.println("ERR line too long");
      used = 0; overflow = false;
    } else if (c != '\r') {
      if (used < sizeof(line)-1) line[used++] = c; else overflow = true;
    }
  }
}
