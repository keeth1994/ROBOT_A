// Controller ESP32: M5Stack Servo2 commands and BMI270 telemetry.
// Install Adafruit PWM Servo Driver Library and SparkFun BMI270 Arduino Library.
// WiFi and WebServer are included with the ESP32 board package.
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include "wifi_config.h"
#include <WebServer.h>
#include <Adafruit_PWMServoDriver.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#include <math.h>

constexpr int SDA_PIN = 21, SCL_PIN = 22;
constexpr uint8_t SERVO_ADDRESS = 0x40;
constexpr uint8_t CHANNELS[] = {0, 1, 2, 3, 8, 9, 10, 11};
constexpr size_t CHANNEL_COUNT = sizeof(CHANNELS) / sizeof(CHANNELS[0]);
constexpr size_t GROUP_SIZE = 4;
// Existing bench limits; not measured robot joint calibration.
constexpr uint16_t CENTER_US = 1500, LEFT_US = 1000, RIGHT_US = 2000;
// Small manual range for initial checks; expand only after checking joint travel.
constexpr uint16_t MANUAL_MIN_US = 1400, MANUAL_MAX_US = 1600;
constexpr uint32_t MOVE_TIME_MS = 1500, UPDATE_MS = 20;
constexpr uint32_t WIFI_TIMEOUT_MS = 3000;
bool wifiWasConnected = false;
IPAddress controllerIp;
uint32_t lastReconnect = 0;
static_assert(MOVE_TIME_MS >= UPDATE_MS, "Move time must be at least one frame.");
static_assert(CHANNEL_COUNT > 0 && GROUP_SIZE > 0, "List at least one channel.");
static_assert(MANUAL_MIN_US <= CENTER_US && CENTER_US <= MANUAL_MAX_US,
              "Manual limits must include centre.");

Adafruit_PWMServoDriver pwm(SERVO_ADDRESS, Wire);
BMI270 imu;
WebServer server(80);
bool servoReady = false, imuReady = false, imuValid = false;
bool imuStreaming = true, faulted = false;
uint32_t lastImuReport = 0, lastImuSuccess = 0, lastHeartbeat = 0;
float usPerTick = 0;
const char *message = "Outputs off. Ready for a test.";

// loop() advances phases without waiting: Stop remains available during motion.
// Phases: centre hold, move left, move right, return centre, hold, gap.
bool running = false, grouped = false, wifiRun = false;
size_t groupFirst = 0, groupCount = 0;
uint8_t phase = 0;
uint32_t phaseStarted = 0, lastServoUpdate = 0;
// Zero means disabled; otherwise this is the last commanded pulse, not feedback.
uint16_t manualPulse[CHANNEL_COUNT] = {};

bool manualActive()
{
  for (size_t i = 0; i < CHANNEL_COUNT; ++i)
    if (manualPulse[i] != 0)
      return true;
  return false;
}


bool allOff()
{
  for (size_t i = 0; i < CHANNEL_COUNT; ++i)
    manualPulse[i] = 0;
  if (!servoReady)
    return false;
  bool ok = true;
  for (uint8_t channel = 0; channel < 16; ++channel)
    if (pwm.setPWM(channel, 0, 4096) != 0)
      ok = false;
  return ok;
}

void servoFault()
{
  running = false;
  wifiRun = false;
  faulted = true;
  allOff(); // Best effort: broken I2C can prevent disabling previous pulses.
  message = "Servo I2C fault. Cut servo power; check wiring and reset.";
  Serial.println(message);
}

bool stopTest(const char *reason)
{
  running = false;
  wifiRun = false;
  // Preserve a latched fault's explanation until reset.
  if (!faulted)
    message = reason;
  if (servoReady && !allOff())
    servoFault();
  return servoReady && !faulted;
}

bool writeGroup(uint16_t pulseUs)
{
  uint16_t ticks = static_cast<uint16_t>(pulseUs / usPerTick + 0.5f);
  for (size_t i = groupFirst; i < groupFirst + groupCount; ++i)
    if (pwm.setPWM(CHANNELS[i], 0, ticks) != 0)
    {
      servoFault();
      return false;
    }
  return true;
}

bool startGroup()
{
  size_t remaining = CHANNEL_COUNT - groupFirst;
  groupCount = grouped && remaining > GROUP_SIZE ? GROUP_SIZE : remaining;
  phase = 0;
  phaseStarted = lastServoUpdate = millis();
  // No shaft-position feedback: the first centre command can jump.
  return writeGroup(CENTER_US);
}

bool startTest(bool groups, bool fromWifi)
{
  if (!servoReady || faulted || running || manualActive() ||
      (fromWifi && WiFi.status() != WL_CONNECTED))
    return false;
  grouped = groups;
  wifiRun = fromWifi;
  lastHeartbeat = millis();
  groupFirst = 0;
  running = true;
  message = groups ? "Testing groups of four." : "Testing all listed servos.";
  return startGroup();
}

void updateMotion()
{
  uint32_t now = millis();
  if (((running && wifiRun) || manualActive()) && WiFi.status() != WL_CONNECTED)
  {
    stopTest("Wi-Fi disconnected. Outputs off.");
    return;
  }
  if (((running && wifiRun) || manualActive()) && now - lastHeartbeat > WIFI_TIMEOUT_MS)
  {
    stopTest("Wi-Fi heartbeat lost. Outputs off.");
    return;
  }
  if (!running)
    return;
  if (now - lastServoUpdate < UPDATE_MS)
    return;
  lastServoUpdate = now;
  uint32_t elapsed = now - phaseStarted;
  if (phase >= 1 && phase <= 3)
  {
    float progress = elapsed >= MOVE_TIME_MS ? 1.0f : float(elapsed) / MOVE_TIME_MS;
    float eased = progress * progress * (3.0f - 2.0f * progress);
    uint16_t from = phase == 1 ? CENTER_US : (phase == 2 ? LEFT_US : RIGHT_US);
    uint16_t to = phase == 1 ? LEFT_US : (phase == 2 ? RIGHT_US : CENTER_US);
    if (!writeGroup(uint16_t(from + (float(to) - from) * eased + 0.5f)))
      return;
    if (elapsed < MOVE_TIME_MS)
      return;
    ++phase;
    phaseStarted = now;
  }
  else if (phase == 0 && elapsed >= 600)
  {
    phase = 1;
    phaseStarted = now;
  }
  else if (phase == 4 && elapsed >= 300)
  {
    for (size_t i = groupFirst; i < groupFirst + groupCount; ++i)
      if (pwm.setPWM(CHANNELS[i], 0, 4096) != 0)
      {
        servoFault();
        return;
      }
    phase = 5;
    phaseStarted = now;
  }
  else if (phase == 5 && elapsed >= 300)
  {
    groupFirst += groupCount;
    if (groupFirst < CHANNEL_COUNT)
      startGroup();
    else
      stopTest("Done. Outputs off.");
  }
}

void beginImu()
{
  const uint8_t addresses[] = {0x68, 0x69};
  for (uint8_t address : addresses)
  {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0 && imu.beginI2C(address, Wire) == BMI2_OK)
    {
      imuReady = true;
      Serial.print("BMI270 ready at 0x");
      Serial.println(address, HEX);
      return;
    }
  }
  Serial.println("BMI270 unavailable. Servo tests remain available.");
}

void serviceImu()
{
  if (!imuReady || millis() - lastImuReport < 500)
    return;
  lastImuReport = millis();
  imuValid = imu.getSensorData() == BMI2_OK;
  if (imuValid)
    imuValid = isfinite(imu.data.accelX) && isfinite(imu.data.accelY) &&
               isfinite(imu.data.accelZ) && isfinite(imu.data.gyroX) &&
               isfinite(imu.data.gyroY) && isfinite(imu.data.gyroZ);
  if (imuValid)
    lastImuSuccess = millis();
  if (!imuStreaming)
    return; // USB printing only; web readings stay live.
  char line[160];
  int length = imuValid ? snprintf(line, sizeof(line),
                                   "IMU a[g] %.3f %.3f %.3f | gyro[deg/s] %.2f %.2f %.2f\n",
                                   imu.data.accelX, imu.data.accelY, imu.data.accelZ,
                                   imu.data.gyroX, imu.data.gyroY, imu.data.gyroZ)
                        : snprintf(line, sizeof(line), "IMU read failed; check wiring/power.\n");
  if (length > 0 && length < int(sizeof(line)) && Serial.availableForWrite() >= length)
    Serial.write(reinterpret_cast<const uint8_t *>(line), length);
}

String channelList(size_t first, size_t count)
{
  String result;
  for (size_t i = first; i < first + count; ++i)
  {
    if (i > first)
      result += ", ";
    result += String(CHANNELS[i]);
  }
  return result;
}

void sendStatus()
{
  bool fresh = imuValid && millis() - lastImuSuccess < 1500;
  String json = "{\"servoReady\":";
  json += servoReady ? "true" : "false";
  json += ",\"faulted\":";
  json += faulted ? "true" : "false";
  json += ",\"running\":";
  json += running ? "true" : "false";
  json += ",\"message\":\"";
  json += message;
  json += "\",\"channels\":\"";
  json += channelList(0, CHANNEL_COUNT);
  json += "\",\"active\":\"";
  json += running ? channelList(groupFirst, groupCount) : String("none");
  json += "\",\"manualActive\":";
  json += manualActive() ? "true" : "false";
  json += ",\"manualMin\":";
  json += String(MANUAL_MIN_US);
  json += ",\"manualMax\":";
  json += String(MANUAL_MAX_US);
  json += ",\"center\":";
  json += String(CENTER_US);
  json += ",\"manual\":[";
  for (size_t i = 0; i < CHANNEL_COUNT; ++i)
  {
    if (i) json += ",";
    json += manualPulse[i] ? String(manualPulse[i]) : String("null");
  }
  json += "]";
  json += ",\"imuReady\":";
  json += imuReady ? "true" : "false";
  json += ",\"imuValid\":";
  json += fresh ? "true" : "false";
  json += ",\"accel\":[";
  json += fresh ? String(imu.data.accelX, 3) : "null";
  json += ",";
  json += fresh ? String(imu.data.accelY, 3) : "null";
  json += ",";
  json += fresh ? String(imu.data.accelZ, 3) : "null";
  json += "],\"gyro\":[";
  json += fresh ? String(imu.data.gyroX, 2) : "null";
  json += ",";
  json += fresh ? String(imu.data.gyroY, 2) : "null";
  json += ",";
  json += fresh ? String(imu.data.gyroZ, 2) : "null";
  json += "]";
  json += "}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

// Reject missing, negative, fractional, overflowing or partly numeric values.
bool readUnsignedArg(const char *name, uint16_t &value)
{
  String arg = server.arg(name);
  if (arg.length() == 0 || arg.length() > 5)
    return false;
  uint32_t parsed = 0;
  for (size_t i = 0; i < arg.length(); ++i)
  {
    if (arg[i] < '0' || arg[i] > '9')
      return false;
    parsed = parsed * 10 + (arg[i] - '0');
  }
  if (parsed > 65535)
    return false;
  value = static_cast<uint16_t>(parsed);
  return true;
}

void handleManual(bool release)
{
  uint16_t channel, pulse = 0;
  if (!readUnsignedArg("channel", channel) ||
      (!release && (!readUnsignedArg("pulse", pulse) ||
                    pulse < MANUAL_MIN_US || pulse > MANUAL_MAX_US)))
  {
    server.send(400, "text/plain", "Invalid channel or pulse; use the manual limits.");
    return;
  }
  size_t index = 0;
  while (index < CHANNEL_COUNT && CHANNELS[index] != channel) ++index;
  if (index == CHANNEL_COUNT)
  {
    server.send(400, "text/plain", "Channel is not configured in CHANNELS.");
    return;
  }
  if (!servoReady || faulted || running || WiFi.status() != WL_CONNECTED)
  {
    server.send(409, "text/plain", "Manual control unavailable. Stop the test or check controller status.");
    return;
  }
  // A newly enabled channel always starts at the explicitly labelled centre.
  if (!release && manualPulse[index] == 0 && pulse != CENTER_US)
  {
    server.send(409, "text/plain", "Enable this servo at centre first.");
    return;
  }
  uint16_t ticks = release ? 4096 : static_cast<uint16_t>(pulse / usPerTick + 0.5f);
  if (pwm.setPWM(channel, 0, ticks) != 0)
  {
    servoFault();
    server.send(503, "text/plain", message);
    return;
  }
  manualPulse[index] = release ? 0 : pulse;
  lastHeartbeat = millis();
  message = manualActive() ? "Manual control. Enabled servos hold their commands." : "Outputs off. Ready for a test.";
  server.send(200, "text/plain", release ? "Servo pulses disabled; power stays on." : "Servo command applied.");
}

void beginWifi()
{
  server.on("/", HTTP_GET, sendStatus); // API only; the PC serves the dashboard.
  server.on("/status", HTTP_GET, sendStatus);
  server.on("/servo", HTTP_POST, []() { handleManual(false); });
  server.on("/release", HTTP_POST, []() { handleManual(true); });
  server.on("/heartbeat", HTTP_POST, []()
            {
      lastHeartbeat = millis();
      server.send(200, "text/plain", "OK"); });
  server.on("/start", HTTP_POST, []()
            {
      String mode = server.arg("mode");
      if (mode != "all" && mode != "groups")
        server.send(400, "text/plain", "Unknown test mode.");
      else if (!startTest(mode == "groups", true))
        server.send(409, "text/plain", manualActive() ? "Release manual servos or press Stop first." : running ? "A test is already running." : message);
      else server.send(200, "text/plain", "Test started."); });
  server.on("/stop", HTTP_POST, []()
            {
      bool ok = stopTest("Stopped. Outputs off.");
      server.send(ok ? 200 : 503, "text/plain", ok ? message : "Cannot confirm outputs off. Cut servo power."); });
  server.onNotFound([]()
                    { server.send(404, "text/plain", "Not found."); });
  WiFi.setHostname(WIFI_HOSTNAME);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.setSleep(false);
  Serial.print("Joining PC hotspot: ");
  Serial.println(WIFI_SSID);
}

void serviceWifi()
{
  bool connected = WiFi.status() == WL_CONNECTED;
  if (wifiWasConnected && (!connected || WiFi.localIP() != controllerIp))
  {
    if ((running && wifiRun) || manualActive())
      stopTest("Wi-Fi connection changed. Outputs off.");
    server.stop();
    MDNS.end();
    wifiWasConnected = false;
    Serial.println("Wi-Fi connection changed; waiting for PC hotspot.");
  }
  if (connected && !wifiWasConnected)
  {
    controllerIp = WiFi.localIP();
    server.begin();
    if (MDNS.begin(WIFI_HOSTNAME))
      MDNS.addService("http", "tcp", 80);
    else
      Serial.println("mDNS unavailable; use the printed IP address.");
    Serial.println("Servo API: http://robot-a-servo.local/status");
    Serial.print("Servo API IP: http://"); Serial.println(controllerIp);
    Serial.println("PC dashboard: http://127.0.0.1:8765/");
    wifiWasConnected = true;
  }
  if (!connected && millis() - lastReconnect >= RECONNECT_INTERVAL_MS)
  {
    lastReconnect = millis();
    WiFi.reconnect();
  }
}

void setup()
{
  Serial.setTxBufferSize(512);
  Serial.begin(9600);
  delay(1000);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Wire.setTimeOut(20);
  Serial.println("I2C scan:");
  for (uint8_t address = 1; address < 127; ++address)
  {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0)
    {
      Serial.print("  Found 0x");
      Serial.println(address, HEX);
    }
  }
  servoReady = pwm.begin();
  if (servoReady)
  {
    if (!allOff())
      servoFault();
    if (!faulted)
    {
      pwm.setPWMFreq(50);
      delay(10);
      usPerTick = (pwm.readPrescale() + 1) * 1000000.0f / pwm.getOscillatorFrequency();
      for (size_t i = 0; i < CHANNEL_COUNT; ++i)
      {
        if (CHANNELS[i] > 15)
          faulted = true;
        for (size_t j = 0; j < i; ++j)
          if (CHANNELS[i] == CHANNELS[j])
            faulted = true;
      }
      if (faulted)
        message = "Invalid or duplicate servo channels. Fix sketch and reset.";
    }
  }
  else
    message = "Servo2 not found. Check power, wiring and address; reset.";
  beginImu();
  beginWifi(); // Show errors even if the sensor/controller is absent.
  Serial.println(message);
  Serial.println("Commands: t=all, g=groups of four, s/x=stop, i=toggle USB IMU printing.");
}

void loop()
{
  // Expire a lost heartbeat before accepting fresh requests after a long stall.
  updateMotion();
  serviceWifi();
  if (wifiWasConnected) server.handleClient();
  while (Serial.available())
  {
    char command = Serial.read();
    if (command == 's' || command == 'S' || command == 'x' || command == 'X')
      stopTest("Stopped. Outputs off.");
    else if (command == 'i' || command == 'I')
      imuStreaming = !imuStreaming;
    else if (command == 't' || command == 'T' || command == 'g' || command == 'G')
    {
      if (!startTest(command == 'g' || command == 'G', false))
        Serial.println("Test rejected: busy, controller missing or faulted.");
    }
  }
  updateMotion();
  serviceImu();
  delay(1);
}
