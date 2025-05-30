#include <WiFi.h>
#include <HTTPClient.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include "esp_camera.h"
#include "driver/i2s.h"

// ==== WiFi Config ====
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";

// ==== Backend Config ====
const char* server_url_audio = "http://YOUR_SERVER_IP:5000/audio-upload";
const char* server_url_gps = "http://YOUR_SERVER_IP:5000/gps-data";

// ==== GPS Config ====
TinyGPSPlus gps;
HardwareSerial gpsSerial(1); // Use UART1

// ==== Audio Config ====
#define I2S_WS  15
#define I2S_SD  32
#define I2S_SCK 14
#define SAMPLE_RATE     (16000)
#define SAMPLE_BITS     (16)
#define I2S_PORT        I2S_NUM_0
#define BUFFER_SIZE     2048

// ==== Camera Config ====
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

void startCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
}

// ==== I2S Audio Setup ====
void startI2S() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = (i2s_bits_per_sample_t)SAMPLE_BITS,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 1024,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
}

void sendAudioData() {
  uint8_t buffer[BUFFER_SIZE];
  size_t bytesRead;

  i2s_read(I2S_PORT, buffer, BUFFER_SIZE, &bytesRead, portMAX_DELAY);

  HTTPClient http;
  http.begin(server_url_audio);
  http.addHeader("Content-Type", "application/octet-stream");
  int httpCode = http.POST(buffer, bytesRead);
  http.end();

  Serial.printf("Sent audio (%d bytes) - Code: %d\n", bytesRead, httpCode);
}

void sendGPSData() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isValid()) {
    float lat = gps.location.lat();
    float lng = gps.location.lng();
    String payload = "{\"lat\": " + String(lat, 6) + ", \"lng\": " + String(lng, 6) + "}";

    HTTPClient http;
    http.begin(server_url_gps);
    http.addHeader("Content-Type", "application/json");
    int httpCode = http.POST(payload);
    http.end();

    Serial.printf("Sent GPS: %s - Code: %d\n", payload.c_str(), httpCode);
  }
}

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 12, 13);  // RX=12, TX=13 (check your GPS pins)

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  startCamera();
  startI2S();
}

unsigned long lastAudioSend = 0;
unsigned long lastGPSSend = 0;

void loop() {
  unsigned long now = millis();

  // Send audio every 5 seconds
  if (now - lastAudioSend > 5000) {
    sendAudioData();
    lastAudioSend = now;
  }

  // Send GPS every 3 seconds
  if (now - lastGPSSend > 3000) {
    sendGPSData();
    lastGPSSend = now;
  }
}
