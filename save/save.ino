#include <Arduino.h>
#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"
#include "SD.h"
#include <driver/i2s.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// ==== CAMERA PINS ====
#define PWDN_GPIO_NUM     32
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

// ==== I2S & GPS ====
#define I2S_PORT I2S_NUM_1
#define I2S_WS   15
#define I2S_SD   2
#define I2S_SCK  14
#define GPS_RX   13
#define GPS_TX   12
#define SD_CS    5

const char* ssid = "PLDTHOMEFIBRUXH9p";
const char* password = "PLDTWIFI3VNXq";
const char* FLASK_URL = "http://192.168.1.49:5000";
const int patientId = 1;
const uint64_t maxVideoSize = 700ULL * 1024 * 1024;

HardwareSerial gpsSerial(2);
TinyGPSPlus gps;
fs::FS* fsUsed = nullptr;

// ==== INIT ====

void initWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("🔌 Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi connected");
}

void initGPS() {
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("✅ GPS initialized");
}

void initSDCard() {
  Serial.println("[SD] Trying SD_MMC (4-bit)");
  if (SD_MMC.begin("/sdcard", false)) { // 4-bit mode
    fsUsed = &SD_MMC;
    Serial.println("[SD] SD_MMC mounted (4-bit mode)");

    uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
    Serial.printf("[SD] Card size: %llu MB\n", cardSize);

    uint8_t cardType = SD_MMC.cardType();
    if (cardType == CARD_NONE) {
      Serial.println("[SD] No SD card detected");
    } else {
      Serial.printf("[SD] Card type: %d\n", cardType);
    }

    return;
  }

  Serial.println("[SD] SD_MMC failed");
}

void initI2SMic() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
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
  Serial.println("✅ I2S mic initialized (I2S_NUM_1)");
}

void initCamera() {
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
  config.jpeg_quality = 10;
  config.fb_count = 1;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("❌ Camera init failed");
    return;
  }
  Serial.println("✅ Camera initialized");
}

// ==== AUDIO ====

void recordWav(const char* filename, int seconds = 7) {
  if (!fsUsed) return;
  const int sample_rate = 16000;
  const int num_samples = sample_rate * seconds;
  File file = fsUsed->open(filename, FILE_WRITE);
  if (!file) return;

  for (int i = 0; i < 44; i++) file.write((uint8_t)0);

  size_t bytes_written = 0;
  int16_t buffer[512];
  for (int i = 0; i < num_samples / 512; i++) {
    size_t bytes_read;
    i2s_read(I2S_PORT, (char *)buffer, sizeof(buffer), &bytes_read, portMAX_DELAY);
    file.write((uint8_t*)buffer, bytes_read);
    bytes_written += bytes_read;
  }

  file.seek(0);
  file.write((const uint8_t*)"RIFF", 4);
  uint32_t file_size = bytes_written + 36;
  file.write((uint8_t*)&file_size, 4);
  file.write((const uint8_t*)"WAVEfmt ", 8);
  uint32_t subchunk1_size = 16;
  uint16_t audio_format = 1, num_channels = 1;
  uint32_t byte_rate = sample_rate * num_channels * 2;
  uint16_t block_align = num_channels * 2, bits_per_sample = 16;
  file.write((uint8_t*)&subchunk1_size, 4);
  file.write((uint8_t*)&audio_format, 2);
  file.write((uint8_t*)&num_channels, 2);
  file.write((uint8_t*)&sample_rate, 4);
  file.write((uint8_t*)&byte_rate, 4);
  file.write((uint8_t*)&block_align, 2);
  file.write((uint8_t*)&bits_per_sample, 2);
  file.write((const uint8_t*)"data", 4);
  file.write((uint8_t*)&bytes_written, 4);
  file.close();
  delay(100); // wait before next read/write

}

// ==== CLOUD ====

String getPresignedUrl(String& outKey) {
  HTTPClient http;
  http.begin(String(FLASK_URL) + "/generate-url");
  int httpCode = http.GET();
  if (httpCode != 200) return "";
  String payload = http.getString();
  StaticJsonDocument<1024> doc;
  if (deserializeJson(doc, payload)) return "";
  outKey = String((const char*)doc["key"]);
  String url = doc["url"];
  url.replace("\\u0026", "&");
  return url;
}

bool uploadWavToS3(const char* localPath, String& uploadedKey) {
  if (!fsUsed) return false;
  uploadedKey = "";
  String presignedUrl = getPresignedUrl(uploadedKey);
  if (presignedUrl == "") return false;
  File file = fsUsed->open(localPath);
  if (!file) return false;
  int size = file.size();
  uint8_t* buffer = (uint8_t*)malloc(size);
  if (!buffer) {file.close(); return false; }
  file.read(buffer, size);
  file.close();
  delay(100); // wait before next read/write
  HTTPClient http;
  http.begin(presignedUrl);
  http.addHeader("Content-Type", "audio/wav");
  int code = http.PUT(buffer, size);
  http.end();
  free(buffer);
  if (code == 200 && fsUsed->exists(localPath)) {
    fsUsed->remove(localPath);
    Serial.println("🗑️ Audio deleted from SD after upload");
  } else Serial.println("⚠️ Upload failed, audio not deleted");
  return (code == 200);
}

void notifyBackend(String key) {
  if (!gps.location.isValid()) return;
  HTTPClient http;
  http.begin(String(FLASK_URL) + "/audio-meta");
  http.addHeader("Content-Type", "application/json");
  String body = "{\"key\":\"" + key + "\",\"lat\":" + String(gps.location.lat(), 6) + ",\"lon\":" + String(gps.location.lng(), 6) + ",\"patientId\":" + String(patientId) + "}";
  http.POST(body);
  http.end();
}

// ==== OFFLINE ====

uint64_t getSDCardUsedBytes() {
  if (!fsUsed) return 0;
  uint64_t total = 0;
  File root = fsUsed->open("/");
  File file;
  while ((file = root.openNextFile())) {
    total += file.size();
    file.close();
    delay(100); // wait before next read/write
  }
  return total;
}

void recordOfflineVideoAudio() {
  if (!fsUsed) return;
  Serial.println("📼 Offline: Recording video + audio to SD");
  char filename[32];
  int fileIndex = 0;
  while (WiFi.status() != WL_CONNECTED && getSDCardUsedBytes() < maxVideoSize) {
    snprintf(filename, sizeof(filename), "/offline_%d.jpg", fileIndex);
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
      File pic = fsUsed->open(filename, FILE_WRITE);
      if (pic) {
        pic.write(fb->buf, fb->len);
        pic.close();
      }
      esp_camera_fb_return(fb);
    }
    snprintf(filename, sizeof(filename), "/offline_%d.wav", fileIndex);
    recordWav(filename, 7);
    fileIndex++;
    delay(1000);
  }
  Serial.println("🛑 Offline recording done.");
}

// ==== SETUP & LOOP ====

void setup() {
  Serial.begin(115200);
  delay(1000);
  initWiFi();
  initGPS();
  initSDCard();
  initI2SMic();
  initCamera();
}

void loop() {
  while (gpsSerial.available()) gps.encode(gpsSerial.read());
  if (WiFi.status() == WL_CONNECTED && fsUsed) {
    recordWav("/audio.wav", 7);
    String key;
    if (uploadWavToS3("/audio.wav", key)) notifyBackend(key);
  } else {
    recordOfflineVideoAudio();
    delay(5000);
    initWiFi();
  }
}

