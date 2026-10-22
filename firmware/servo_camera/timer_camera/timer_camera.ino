// Upload to M5Stack TimerCamera-X, NOT to the servo ESP32.
// Board: M5TimerCAM; PSRAM: Enabled. Arduino ESP32 core 3.3.12.
// esp_camera, WiFi, ESPmDNS and esp_http_server come with the ESP32 core.
#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include "wifi_config.h"
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include <atomic>
#include <mutex>

constexpr int BAT_HOLD_PIN = 33, STATUS_LED_PIN = 2;
constexpr uint8_t DEFAULT_FPS = 10, DEFAULT_JPEG_QUALITY = 12;
constexpr bool FLIP_VERTICAL = true;
constexpr bool MIRROR_HORIZONTAL = false;

httpd_handle_t pageServer = nullptr, streamServer = nullptr;
std::atomic<bool> cameraInitialized{false};
bool wasConnected = false;
uint32_t lastReconnect = 0;
IPAddress cameraIp;

// Capture and sensor changes run in different HTTP tasks. Never change sensor
// registers while a framebuffer is in use. State lock is only held for copies.
std::mutex captureMutex, stateMutex;
struct CameraState
{
  const char *resolution = "vga";
  uint16_t width = 640, height = 480;
  uint8_t targetFps = DEFAULT_FPS, quality = DEFAULT_JPEG_QUALITY;
  bool streaming = false, hasFrame = false;
  float sentFps = 0;
  uint32_t lastSend = 0, frameAgeMs = 0;
  size_t frameBytes = 0;
} cameraState;

struct StreamSession
{
  StreamSession()
  {
    std::lock_guard<std::mutex> lock(stateMutex);
    cameraState.streaming = true;
    cameraState.hasFrame = false;
    cameraState.sentFps = 0;
  }
  ~StreamSession()
  {
    std::lock_guard<std::mutex> lock(stateMutex);
    cameraState.streaming = false;
    cameraState.sentFps = 0;
  }
};

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
  // Allocate buffers for the largest selectable size, then start at VGA.
  config.frame_size = FRAMESIZE_SVGA;
  config.jpeg_quality = DEFAULT_JPEG_QUALITY; // Lower is better JPEG quality.
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
      sensor->set_hmirror(sensor, MIRROR_HORIZONTAL) != 0 ||
      sensor->set_framesize(sensor, FRAMESIZE_VGA) != 0)
  {
    Serial.println("Camera sensor configuration failed.");
    esp_camera_deinit();
    return false;
  }
  return true;
}

esp_err_t statusHandler(httpd_req_t *req)
{
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  bool ready = cameraInitialized && streamServer != nullptr;
  CameraState state;
  {
    std::lock_guard<std::mutex> lock(stateMutex);
    state = cameraState;
  }
  uint32_t age = millis() - state.lastSend;
  float fps = state.streaming && state.hasFrame && age < 1500 ? state.sentFps : 0;
  char lastSend[24], frameAge[24];
  if (state.hasFrame)
  {
    snprintf(lastSend, sizeof(lastSend), "%lu", static_cast<unsigned long>(age));
    snprintf(frameAge, sizeof(frameAge), "%lu", static_cast<unsigned long>(state.frameAgeMs));
  }
  else
  {
    strcpy(lastSend, "null"); strcpy(frameAge, "null");
  }
  char body[600];
  snprintf(body, sizeof(body),
    "{\"ready\":%s,\"error\":\"%s\",\"resolution\":\"%s\",\"width\":%u,\"height\":%u,"
    "\"targetFps\":%u,\"quality\":%u,\"streaming\":%s,\"sentFps\":%.1f,"
    "\"lastSendAgoMs\":%s,\"frameAgeAtSendMs\":%s,\"frameBytes\":%u}",
    ready ? "true" : "false", ready ? "" : "Camera or stream server failed. Check Serial Monitor at 115200 baud, board selection and PSRAM, then reset.",
    state.resolution, state.width, state.height, state.targetFps, state.quality,
    state.streaming ? "true" : "false", fps, lastSend, frameAge,
    static_cast<unsigned>(state.frameBytes));
  return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

esp_err_t settingsError(httpd_req_t *req, const char *status, const char *message)
{
  httpd_resp_set_status(req, status);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  char body[200];
  snprintf(body, sizeof(body), "{\"error\":\"%s\"}", message);
  return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

esp_err_t settingsHandler(httpd_req_t *req)
{
  char body[128], size[8], fpsText[4], qualityText[4];
  if (req->content_len <= 0 || req->content_len >= sizeof(body))
    return settingsError(req, "400 Bad Request", "Invalid settings body.");
  size_t received = 0;
  while (received < req->content_len)
  {
    int count = httpd_req_recv(req, body + received, req->content_len - received);
    if (count <= 0)
      return settingsError(req, "400 Bad Request", "Incomplete settings body.");
    received += count;
  }
  body[received] = '\0';
  if (httpd_query_key_value(body, "resolution", size, sizeof(size)) != ESP_OK ||
      httpd_query_key_value(body, "fps", fpsText, sizeof(fpsText)) != ESP_OK ||
      httpd_query_key_value(body, "quality", qualityText, sizeof(qualityText)) != ESP_OK)
    return settingsError(req, "400 Bad Request", "Resolution, fps and quality are required.");
  framesize_t frameSize;
  uint16_t width, height;
  const char *resolution;
  if (strcmp(size, "qvga") == 0) { frameSize = FRAMESIZE_QVGA; width = 320; height = 240; resolution = "qvga"; }
  else if (strcmp(size, "vga") == 0) { frameSize = FRAMESIZE_VGA; width = 640; height = 480; resolution = "vga"; }
  else if (strcmp(size, "svga") == 0) { frameSize = FRAMESIZE_SVGA; width = 800; height = 600; resolution = "svga"; }
  else return settingsError(req, "400 Bad Request", "Unsupported resolution.");
  uint8_t fps = strcmp(fpsText, "5") == 0 ? 5 : strcmp(fpsText, "10") == 0 ? 10 : strcmp(fpsText, "15") == 0 ? 15 : 0;
  uint8_t quality = strcmp(qualityText, "12") == 0 ? 12 : strcmp(qualityText, "18") == 0 ? 18 : strcmp(qualityText, "24") == 0 ? 24 : 0;
  if (!fps || !quality)
    return settingsError(req, "400 Bad Request", "Unsupported fps or JPEG quality.");
  if (!cameraInitialized)
    return settingsError(req, "503 Service Unavailable", "Camera unavailable. Reset after checking setup.");
  std::unique_lock<std::mutex> capture(captureMutex, std::try_to_lock);
  if (!capture.owns_lock())
    return settingsError(req, "409 Conflict", "Camera busy. Hide video and retry.");
  CameraState previous;
  {
    std::lock_guard<std::mutex> lock(stateMutex);
    previous = cameraState;
  }
  sensor_t *sensor = esp_camera_sensor_get();
  framesize_t oldSize = previous.width == 320 ? FRAMESIZE_QVGA : previous.width == 640 ? FRAMESIZE_VGA : FRAMESIZE_SVGA;
  if (!sensor || sensor->set_framesize(sensor, frameSize) != 0 || sensor->set_quality(sensor, quality) != 0)
  {
    bool restored = sensor && sensor->set_framesize(sensor, oldSize) == 0 && sensor->set_quality(sensor, previous.quality) == 0;
    if (!restored) cameraInitialized = false;
    return settingsError(req, "500 Internal Server Error", restored ? "Sensor rejected settings. Previous settings restored." : "Sensor configuration failed. Reset the camera.");
  }
  // Discard queued/in-flight images captured with the old settings.
  for (int i = 0; i < 3; ++i)
  {
    camera_fb_t *frame = esp_camera_fb_get();
    if (frame) esp_camera_fb_return(frame);
  }
  {
    std::lock_guard<std::mutex> lock(stateMutex);
    cameraState.resolution = resolution;
    cameraState.width = width; cameraState.height = height;
    cameraState.targetFps = fps; cameraState.quality = quality;
    cameraState.sentFps = 0; cameraState.hasFrame = false;
  }
  return statusHandler(req);
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
  StreamSession session;
  uint32_t windowStarted = millis(), windowFrames = 0;
  // This server handles one viewer. Page/status live on a separate HTTP server,
  // so a long stream never prevents camera health requests.
  while (WiFi.status() == WL_CONNECTED)
  {
    uint32_t started = millis();
    std::unique_lock<std::mutex> capture(captureMutex);
    if (!cameraInitialized) return ESP_FAIL;
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
    uint8_t targetFps;
    {
      std::lock_guard<std::mutex> lock(stateMutex);
      targetFps = cameraState.targetFps;
      if (result == ESP_OK)
      {
        uint32_t now = millis();
        ++windowFrames;
        if (now - windowStarted >= 1000)
        {
          cameraState.sentFps = windowFrames * 1000.0f / (now - windowStarted);
          windowStarted = now; windowFrames = 0;
        }
        int64_t capturedUs = int64_t(frame->timestamp.tv_sec) * 1000000 + frame->timestamp.tv_usec;
        int64_t ageUs = esp_timer_get_time() - capturedUs;
        cameraState.frameAgeMs = ageUs > 0 ? static_cast<uint32_t>(ageUs / 1000) : 0;
        cameraState.frameBytes = frame->len;
        cameraState.lastSend = now; cameraState.hasFrame = true;
      }
    }
    esp_camera_fb_return(frame); // Also release buffers after a disconnected viewer.
    capture.unlock();
    if (result != ESP_OK) return result;
    uint32_t elapsed = millis() - started;
    uint32_t interval = (1000 + targetFps - 1) / targetFps;
    if (elapsed < interval) delay(interval - elapsed);
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
  page.uri = "/"; page.method = HTTP_GET; page.handler = statusHandler; // API only; HTML lives on the PC.
  httpd_uri_t status = {};
  status.uri = "/status"; status.method = HTTP_GET; status.handler = statusHandler;
  httpd_uri_t settings = {};
  settings.uri = "/settings"; settings.method = HTTP_POST; settings.handler = settingsHandler;
  if (httpd_register_uri_handler(pageServer, &page) == ESP_OK &&
      httpd_register_uri_handler(pageServer, &status) == ESP_OK &&
      httpd_register_uri_handler(pageServer, &settings) == ESP_OK) return true;
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
  WiFi.setHostname(WIFI_HOSTNAME);
  WiFi.mode(WIFI_STA); // Initialize the network stack before starting HTTP servers.
  WiFi.setAutoReconnect(true);
  if (!beginPageServer()) Serial.println("Camera page server failed.");
  if (cameraInitialized && !beginStreamServer()) Serial.println("Stream server failed.");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.setSleep(false);
  Serial.print("Joining PC hotspot: "); Serial.println(WIFI_SSID);
}

void loop()
{
  bool connected = WiFi.status() == WL_CONNECTED;
  uint32_t now = millis();
  digitalWrite(STATUS_LED_PIN, connected ? HIGH : ((now / 500) % 2));
  if (wasConnected && (!connected || WiFi.localIP() != cameraIp))
  {
    MDNS.end();
    wasConnected = false;
  }
  if (connected)
  {
    if (!wasConnected)
    {
      cameraIp = WiFi.localIP();
      if (MDNS.begin(WIFI_HOSTNAME))
        MDNS.addService("http", "tcp", 80);
      else
        Serial.println("mDNS unavailable; use the printed IP address.");
      Serial.println("Camera API: http://robot-a-camera.local/status");
      Serial.print("Camera API IP: http://"); Serial.println(cameraIp);
      Serial.println("PC dashboard: http://127.0.0.1:8765/");
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
