// ESP32 + M5Stack Servo2 + BMI270 bench test.
// Install Adafruit PWM Servo Driver Library and SparkFun BMI270 Arduino Library.
// WiFi and WebServer are included with the ESP32 board package.
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
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
constexpr char WIFI_SSID[] = "ROBOT-A-Test";
constexpr char WIFI_PASSWORD[] = "robotatest";
// Camera uses DHCP and announces its address every 5 s. Never a servo heartbeat.
constexpr uint32_t CAMERA_TIMEOUT_MS = 15000;
IPAddress cameraIp;
bool cameraSeen = false, cameraReady = false;
uint32_t lastCameraReport = 0;
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

const char PAGE[] PROGMEM = R"HTML(<!doctype html>
  <html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Robot A bench test</title>
  <style>
  body{font:17px system-ui,sans-serif;background:#101820;color:#edf3f7;max-width:620px;margin:auto;padding:24px}
  h1{font-size:27px}button{font:inherit;border:0;border-radius:10px;padding:17px;margin:6px 0;width:100%;cursor:pointer;background:#6fe0bc;color:#101820;font-weight:650}
  input[type=range]{width:100%;min-height:44px}.servo{border-top:1px solid #405362;padding:16px 0}.servo button{padding:10px}button:disabled{opacity:.4;cursor:default}#stop{background:#ff706f}section{background:#1c2935;border-radius:14px;padding:18px;margin:20px 0}pre{white-space:pre-wrap;font-size:15px}small{color:#c2cfd8}#connection{font-weight:600}
  </style><h1>Robot A bench test</h1><p id="connection">Connecting...</p>
  <section><p id="state">Reading controller status...</p>
  <button class="start" data-mode="all" disabled>All servos together</button>
  <button class="start" data-mode="groups" disabled>Four servos at a time</button>
  <button id="stop">Stop / disable pulses</button>
  <p id="channels"></p><small>Support the robot. Initial centering may jump. Stop disables signals; servo power stays on.</small></section>
  <section><h2>TimerCamera-X</h2><p id="cameraState">Waiting for camera to join ROBOT-A-Test...</p>
  <button id="videoStart" disabled>Show video</button><button id="videoStop" disabled>Hide video</button>
  <p><a id="cameraLink" hidden target="_blank" rel="noopener" style="color:#6fe0bc">Open camera in a separate tab</a></p>
  <img id="cameraFeed" hidden alt="TimerCamera-X video" style="width:100%;border-radius:10px">
  <small>Keep this controller page visible during servo control. Video may freeze or lag if Wi-Fi is interrupted.</small></section>
  <section><h2>Individual servos</h2><p>Enable a servo at centre, then drag its slider. Release the slider to apply the command.</p>
  <small>Values are pulse widths in microseconds, not measured angles. Initial enabling can jump. Release disables pulses; power stays on. Automatic sweeps still use 1000–2000 µs.</small><div id="servos"></div></section>
  <section><h2>BMI270</h2><pre id="imu">Waiting for readings...</pre><small>Sensor axes X / Y / Z. Acceleration includes gravity; gyro measures rotation speed.</small></section>
  <p id="result" role="status"></p><small>Keep this page open during Wi-Fi tests or manual control. If its heartbeat is lost for 3 seconds, the test stops. No automatic restart.</small>
  <script>
  const el=id=>document.getElementById(id);
  let busy=false, status=null, connected=false;
  const rows=new Map();
  let cameraBase='', videoShown=false;
  function hideVideo(){
    el('cameraFeed').removeAttribute('src');el('cameraFeed').hidden=true;
    videoShown=false;el('videoStop').disabled=true;
  }
  function renderCamera(camera){
    const base=camera.online&&camera.ready?camera.url:'';
    if(base!==cameraBase)hideVideo();
    cameraBase=base;
    el('cameraLink').hidden=!cameraBase;el('cameraLink').href=cameraBase;
    el('videoStart').disabled=!cameraBase;
    el('cameraState').textContent=!camera.online?'Camera offline. Power on the TimerCamera-X and wait for it to join Wi-Fi.':!camera.ready?'Camera joined Wi-Fi but initialization failed. Check its Serial Monitor at 115200 baud.':videoShown?'Video requested. If frozen, hide it and show it again.':'Camera connected. Press Show video.';
  }
  el('videoStart').onclick=()=>{
    if(!cameraBase)return;
    el('cameraFeed').src=cameraBase.replace(/\/$/,'')+':81/stream?t='+Date.now();
    el('cameraFeed').hidden=false;videoShown=true;el('videoStop').disabled=false;
    el('cameraState').textContent='Video requested. If frozen, hide it and show it again.';
  };
  el('videoStop').onclick=()=>{hideVideo();el('cameraState').textContent='Video hidden.';};
  el('cameraFeed').onerror=()=>{hideVideo();el('cameraState').textContent='Video unavailable. Close other video tabs and try Show video again.';};
  async function request(path,body){
    const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:body||'',cache:'no-store',signal:AbortSignal.timeout(1500)});
    const text=await r.text();if(!r.ok)throw Error(text);return text;
  }
  function controls(){
    const blocked=!connected||!status||!status.servoReady||status.faulted||status.running||busy;
    document.querySelectorAll('.start').forEach(b=>b.disabled=blocked||status.manualActive);
    rows.forEach(row=>{
      row.enable.disabled=blocked||row.active;
      row.slider.disabled=blocked||!row.active;
      row.release.disabled=blocked||!row.active;
    });
  }
  async function command(path,body){
    if(busy)return;
    busy=true;controls();
    try{el('result').textContent=await request(path,body);}
    catch(e){el('result').textContent='Command not confirmed: '+e.message+' Use Stop or cut servo power if needed.';}
    finally{busy=false;controls();}
  }
  function renderServos(s){
    s.channels.split(',').map(c=>c.trim()).forEach((channel,i)=>{
      let row=rows.get(channel);
      if(!row){
        const box=document.createElement('div');box.className='servo';
        const label=document.createElement('label');label.htmlFor='servo-'+channel;
        const detail=document.createElement('p');
        const slider=document.createElement('input');slider.type='range';slider.id=label.htmlFor;slider.step=1;
        const enable=document.createElement('button');enable.textContent='Enable at '+s.center+' µs';
        const release=document.createElement('button');release.textContent='Release servo';
        box.append(label,slider,detail,enable,release);el('servos').append(box);
        row={label,slider,detail,enable,release,active:false};rows.set(channel,row);
        const show=()=>label.textContent='Channel '+channel+': '+slider.value+' µs selected';
        slider.oninput=show;
        slider.onchange=()=>command('/servo','channel='+channel+'&pulse='+slider.value);
        enable.onclick=()=>command('/servo','channel='+channel+'&pulse='+status.center);
        release.onclick=()=>command('/release','channel='+channel);
        row.show=show;
      }
      const active=s.manual[i]!==null;
      row.slider.min=s.manualMin;row.slider.max=s.manualMax;
      // Do not reset a slider under the user's finger on every status poll.
      if(active!==row.active||!active)row.slider.value=active?s.manual[i]:s.center;
      row.active=active;row.show();
      row.detail.textContent=active?'Enabled — commanded '+s.manual[i]+' µs':'Pulses off';
    });
  }
  async function poll(){
    try{
      await request('/heartbeat');
      const r=await fetch('/status',{cache:'no-store',signal:AbortSignal.timeout(1500)});
      if(!r.ok)throw Error('Status unavailable');
      status=await r.json();connected=true;
      el('connection').textContent='Connected to robot';
      el('state').textContent=status.message+(status.running?' — test running':'');
      el('channels').textContent='Channels: '+status.channels+(status.running?' | Active: '+status.active:'');
      renderServos(status);
      renderCamera(status.camera);
      el('imu').textContent=!status.imuReady?'BMI270 not found. Check wiring and reset.':!status.imuValid?'IMU reading unavailable or stale.':'Acceleration [g]   '+status.accel.map(v=>v.toFixed(3)).join(' / ')+'\nGyro [deg/s]      '+status.gyro.map(v=>v.toFixed(2)).join(' / ');
    }catch(e){
      connected=false;el('connection').textContent='Connection lost — check Wi-Fi';el('imu').textContent='No live readings';
      hideVideo();cameraBase='';el('videoStart').disabled=true;el('cameraLink').hidden=true;
      el('cameraState').textContent='Controller connection lost. Video hidden.';
    }finally{controls();setTimeout(poll,500);}
  }
  document.querySelectorAll('.start').forEach(b=>b.onclick=()=>command('/start','mode='+b.dataset.mode));
  el('stop').onclick=async()=>{
    try{el('result').textContent=await request('/stop');}
    catch(e){el('result').textContent='Stop NOT confirmed. Use the physical power switch if needed.';}
  };
  poll();
  </script></html>)HTML";

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
  if (!servoReady || faulted || running || manualActive())
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
  // No HTTP request to the camera here: an absent/slow camera must never block
  // servo servicing.
  bool cameraOnline = cameraSeen && millis() - lastCameraReport < CAMERA_TIMEOUT_MS;
  json += ",\"camera\":{\"online\":";
  json += cameraOnline ? "true" : "false";
  json += ",\"ready\":";
  json += cameraOnline && cameraReady ? "true" : "false";
  json += ",\"url\":\"";
  if (cameraOnline)
    json += String("http://") + cameraIp.toString();
  json += "\"}}";
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
  if (!servoReady || faulted || running)
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
  server.on("/", HTTP_GET, []()
            { server.send_P(200, "text/html", PAGE); });
  server.on("/status", HTTP_GET, sendStatus);
  server.on("/camera/register", HTTP_POST, []()
            {
      String ready = server.arg("ready");
      IPAddress peer = server.client().remoteIP();
      // Learn the actual DHCP address; do not accept a supplied URL or host.
      if ((ready != "0" && ready != "1") || peer[0] != 192 ||
          peer[1] != 168 || peer[2] != 4 || peer[3] < 2 || peer[3] > 254)
      {
        server.send(400, "text/plain", "Invalid camera registration.");
        return;
      }
      cameraIp = peer;
      cameraReady = ready == "1";
      cameraSeen = true;
      lastCameraReport = millis();
      // Deliberately do not update lastHeartbeat: camera cannot keep servos on.
      server.send(200, "text/plain", "Camera registered."); });
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
  WiFi.mode(WIFI_AP);
  IPAddress ip(192, 168, 4, 1);
  if (!WiFi.softAPConfig(ip, ip, IPAddress(255, 255, 255, 0)) ||
      !WiFi.softAP(WIFI_SSID, WIFI_PASSWORD, 1, false, 4))
  {
    Serial.println("Wi-Fi startup failed. USB serial control remains available.");
    return;
  }
  server.begin();
  Serial.print("Wi-Fi: ");
  Serial.println(WIFI_SSID);
  Serial.print("Password: ");
  Serial.println(WIFI_PASSWORD);
  Serial.println("Open http://192.168.4.1");
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
  server.handleClient();
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
