// Upload to M5Stack TimerCamera-X, NOT to the servo ESP32.
// Board: M5TimerCAM; PSRAM: Enabled. Arduino ESP32 core 3.3.12.
// esp_camera, WiFi, HTTPClient and esp_http_server come with the ESP32 core.
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "esp_camera.h"
#include "esp_http_server.h"

// Match the servo_test hotspot credentials if you change them.
constexpr char WIFI_SSID[] = "ROBOT-A-Test";
constexpr char WIFI_PASSWORD[] = "robotatest";
constexpr int BAT_HOLD_PIN = 33, STATUS_LED_PIN = 2;
constexpr uint32_t REGISTER_INTERVAL_MS = 5000, RECONNECT_INTERVAL_MS = 10000;
constexpr uint32_t FRAME_INTERVAL_MS = 200; // At most 5 fps; leave airtime for controls.
constexpr bool FLIP_VERTICAL = true;
constexpr bool MIRROR_HORIZONTAL = false;

httpd_handle_t pageServer = nullptr, streamServer = nullptr;
bool cameraInitialized = false;
bool wasConnected = false;
uint32_t lastRegistration = 0, lastReconnect = 0;

const char PAGE[] PROGMEM = R"HTML(<!doctype html>
<html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Robot A camera</title>
<style>body{font:17px system-ui;background:#101820;color:#edf3f7;max-width:720px;margin:auto;padding:24px}button{font:inherit;padding:14px;margin:6px}img{width:100%;border-radius:12px}a{color:#6fe0bc}</style>
<h1>TimerCamera-X</h1><p id="state">Reading camera status...</p>
<button id="start" disabled>Show video</button><button id="stop">Hide video</button>
<p><a href="http://192.168.4.1/">Robot servo controls</a></p>
<img id="feed" hidden alt="TimerCamera-X video">
<p>One video viewer at a time. Keep the robot controller page visible during servo control; this camera page does not send servo heartbeats.</p>
<script>
const el=id=>document.getElementById(id);
function stop(){el('feed').removeAttribute('src');el('feed').hidden=true;}
el('start').onclick=()=>{el('feed').src='http://'+location.hostname+':81/stream?t='+Date.now();el('feed').hidden=false;el('state').textContent='Video requested. If frozen, hide it and show it again.';};
el('stop').onclick=()=>{stop();el('state').textContent='Video hidden.';};
el('feed').onerror=()=>{stop();el('state').textContent='Video unavailable. Close other video tabs and try again.';};
async function status(){try{const r=await fetch('/status',{cache:'no-store',signal:AbortSignal.timeout(2000)});if(!r.ok)throw Error();const s=await r.json();el('start').disabled=!s.ready;if(!s.ready){stop();el('state').textContent=s.error;}else if(el('feed').hidden){el('state').textContent='Camera ready — 320 × 240, up to 5 fps.';}}catch(e){stop();el('start').disabled=true;el('state').textContent='Camera connection lost.';}finally{setTimeout(status,2000);}}
status();
</script></html>)HTML";

bool initializeCamera()
{
  if (!psramFound())
  {
    Serial.println("PSRAM unavailable. Select M5TimerCAM, PSRAM Enabled, and reset.");
    return false;
  }
  // TimerCamera-X OV3660 pin map from M5Stack documentation (not AI Thinker).
  camera_config_t config = {};
  config.pin_pwdn = -1;
  config.pin_reset = 15;
  config.pin_xclk = 27;
  config.pin_sccb_sda = 25;
  config.pin_sccb_scl = 23;
  config.pin_d0 = 32;
  config.pin_d1 = 35;
  config.pin_d2 = 34;
  config.pin_d3 = 5;
  config.pin_d4 = 39;
  config.pin_d5 = 18;
  config.pin_d6 = 36;
  config.pin_d7 = 19;
  config.pin_vsync = 22;
  config.pin_href = 26;
  config.pin_pclk = 21;
  config.xclk_freq_hz = 20000000;
  config.ledc_timer = LEDC_TIMER_0;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12; // Lower number gives better JPEG quality.
  config.fb_count = 2;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;
  esp_err_t error = esp_camera_init(&config);
  if (error != ESP_OK)
  {
    Serial.printf("Camera initialization failed: 0x%x\n", error);
    return false;
  }
  sensor_t *sensor = esp_camera_sensor_get();
  if (!sensor || sensor->set_vflip(sensor, FLIP_VERTICAL) != 0 ||
      sensor->set_hmirror(sensor, MIRROR_HORIZONTAL) != 0)
  {
    Serial.println("Camera sensor configuration failed.");
    esp_camera_deinit();
    return false;
  }
  return true;
}

esp_err_t pageHandler(httpd_req_t *req)
{
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, PAGE, HTTPD_RESP_USE_STRLEN);
}

esp_err_t statusHandler(httpd_req_t *req)
{
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  bool ready = cameraInitialized && streamServer != nullptr;
  return httpd_resp_send(req, ready ? "{\"ready\":true,\"error\":\"\"}"
    : "{\"ready\":false,\"error\":\"Camera or stream server failed. Check Serial Monitor at 115200 baud, board selection and PSRAM, then reset.\"}", HTTPD_RESP_USE_STRLEN);
}

esp_err_t streamHandler(httpd_req_t *req)
{
  if (!cameraInitialized)
  {
    httpd_resp_set_status(req, "503 Service Unavailable");
    return httpd_resp_send(req, "Camera unavailable.", HTTPD_RESP_USE_STRLEN);
  }
  esp_err_t result = httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=robot-a-frame");
  if (result != ESP_OK) return result;
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  // This server handles one viewer. Page/status live on a separate HTTP server,
  // so a long stream never prevents camera health requests or Wi-Fi registration.
  while (WiFi.status() == WL_CONNECTED)
  {
    uint32_t started = millis();
    camera_fb_t *frame = esp_camera_fb_get();
    if (!frame) return ESP_FAIL;
    if (frame->format != PIXFORMAT_JPEG)
    {
      esp_camera_fb_return(frame);
      return ESP_FAIL;
    }
    char header[100];
    int length = snprintf(header, sizeof(header),
      "\r\n--robot-a-frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
      static_cast<unsigned>(frame->len));
    if (length <= 0 || length >= int(sizeof(header)))
      result = ESP_FAIL;
    else
      result = httpd_resp_send_chunk(req, header, length);
    if (result == ESP_OK)
      result = httpd_resp_send_chunk(req, reinterpret_cast<const char *>(frame->buf), frame->len);
    esp_camera_fb_return(frame); // Also release buffers after a disconnected viewer.
    if (result != ESP_OK) return result;
    uint32_t elapsed = millis() - started;
    if (elapsed < FRAME_INTERVAL_MS) delay(FRAME_INTERVAL_MS - elapsed);
  }
  return ESP_FAIL; // Close the HTTP session after Wi-Fi loss.
}

bool beginPageServer()
{
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.max_open_sockets = 3;
  config.lru_purge_enable = true;
  config.send_wait_timeout = 2;
  config.recv_wait_timeout = 2;
  if (httpd_start(&pageServer, &config) != ESP_OK) return false;
  httpd_uri_t page = {};
  page.uri = "/"; page.method = HTTP_GET; page.handler = pageHandler;
  httpd_uri_t status = {};
  status.uri = "/status"; status.method = HTTP_GET; status.handler = statusHandler;
  if (httpd_register_uri_handler(pageServer, &page) == ESP_OK &&
      httpd_register_uri_handler(pageServer, &status) == ESP_OK) return true;
  httpd_stop(pageServer); pageServer = nullptr;
  return false;
}

bool beginStreamServer()
{
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 81;
  config.ctrl_port += 1; // Must differ from the port-80 HTTP server's control port.
  config.max_open_sockets = 1;
  config.send_wait_timeout = 2;
  config.recv_wait_timeout = 2;
  httpd_uri_t stream = {};
  stream.uri = "/stream"; stream.method = HTTP_GET; stream.handler = streamHandler;
  if (httpd_start(&streamServer, &config) != ESP_OK) return false;
  if (httpd_register_uri_handler(streamServer, &stream) == ESP_OK) return true;
  httpd_stop(streamServer); streamServer = nullptr;
  return false;
}

void registerWithController()
{
  // AP supplies the gateway via DHCP. No fixed camera address to clash with PCs.
  String url = String("http://") + WiFi.gatewayIP().toString() + "/camera/register";
  WiFiClient client;
  HTTPClient http;
  http.setConnectTimeout(500);
  http.setTimeout(500);
  if (!http.begin(client, url)) return;
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  int code = http.POST(cameraInitialized && pageServer && streamServer ? "ready=1" : "ready=0");
  http.end();
  if (code != 200) Serial.printf("Camera registration failed (%d); retrying in 5 s.\n", code);
}

void setup()
{
  // External-power operation: do not latch the internal battery supply on.
  // GPIO33 LOW releases battery hold (M5Stack Power_Class::powerOff).
  // External 5V still powers the camera; unplugging it should remove main power.
  digitalWrite(BAT_HOLD_PIN, LOW);
  pinMode(BAT_HOLD_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  Serial.begin(115200);
  delay(500);
  cameraInitialized = initializeCamera();
  WiFi.mode(WIFI_STA); // Initialize the network stack before starting HTTP servers.
  WiFi.setHostname("robot-a-camera");
  WiFi.setAutoReconnect(true);
  if (!beginPageServer()) Serial.println("Camera page server failed.");
  if (cameraInitialized && !beginStreamServer()) Serial.println("Stream server failed.");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.setSleep(false);
  Serial.println("Joining ROBOT-A-Test. Power on the servo controller first.");
}

void loop()
{
  bool connected = WiFi.status() == WL_CONNECTED;
  uint32_t now = millis();
  digitalWrite(STATUS_LED_PIN, connected ? HIGH : ((now / 500) % 2));
  if (connected)
  {
    if (!wasConnected)
    {
      Serial.print("Camera page: http://"); Serial.println(WiFi.localIP());
      Serial.println("Open http://192.168.4.1 on your computer for servo controls and video.");
    }
    if (!wasConnected || now - lastRegistration >= REGISTER_INTERVAL_MS)
    {
      lastRegistration = now;
      registerWithController(); // Never sends /heartbeat or a servo command.
    }
  }
  else if (now - lastReconnect >= RECONNECT_INTERVAL_MS)
  {
    lastReconnect = now;
    WiFi.reconnect();
  }
  wasConnected = connected;
  delay(20);
}
