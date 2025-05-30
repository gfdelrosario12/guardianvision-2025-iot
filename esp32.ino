#include "esp_camera.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <driver/i2s.h>

// === WiFi Credentials ===
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";

// === AWS Endpoint URLs ===
const char* AUDIO_ENDPOINT = "http://your-aws-endpoint/audio";
const char* GPS_ENDPOINT = "http://your-aws-endpoint/gps";

// === Camera Pins === (adjust to your board)
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

// === I2S Configuration for Microphone ===
#define I2S_WS  15
#define I2S_SD  13
#define I2S_SCK 14

#define SAMPLE_RATE   16000
#define I2S_BUFFER_SIZE 1024

// === GPS Serial Config ===
HardwareSerial GPS_Serial(2);  // Use UART2: pins 16 (RX), 17 (TX)
TinyGPSPlus gps;

// === Globals ===
WiFiServer videoServer(81);  // MJPEG stream port

// I2S setup function
void i2s_init() {
  const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = I2S_BUFFER_SIZE,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
}

// Camera init
void init_camera() {
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
    while (true) { delay(1000); }
  }
}

// MJPEG streaming to clients
void startCameraServer() {
  videoServer.begin();
}

void handleMJPEGClient(WiFiClient client) {
  String response = 
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
  client.print(response);

  while (client.connected()) {
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      continue;
    }

    client.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", fb->len);
    client.write(fb->buf, fb->len);
    client.write("\r\n");
    esp_camera_fb_return(fb);

    if (!client.connected()) break;
    delay(33); // ~30 fps
  }
}

// Send audio chunk via HTTP POST
void sendAudioChunk(uint8_t* buffer, size_t len) {
  HTTPClient http;
  http.begin(AUDIO_ENDPOINT);
  http.addHeader("Content-Type", "application/octet-stream");
  int code = http.POST(buffer, len);
  if (code > 0) {
    Serial.printf("Sent audio chunk, response: %d\n", code);
  } else {
    Serial.printf("Failed to send audio chunk\n");
  }
  http.end();
}

// Send GPS data as JSON via HTTP POST
void sendGPSData(double lat, double lng) {
  HTTPClient http;
  http.begin(GPS_ENDPOINT);
  http.addHeader("Content-Type", "application/json");
  String payload = "{\"latitude\": " + String(lat, 6) + ", \"longitude\": " + String(lng, 6) + "}";
  int code = http.POST(payload);
  if (code > 0) {
    Serial.printf("Sent GPS data, response: %d\n", code);
  } else {
    Serial.printf("Failed to send GPS data\n");
  }
  http.end();
}

void setup() {
  Serial.begin(115200);

  // Connect WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected!");

  // Initialize camera and server
  init_camera();
  startCameraServer();

  // Initialize I2S mic
  i2s_init();

  // Initialize GPS UART
  GPS_Serial.begin(9600, SERIAL_8N1, 16, 17);
}

void loop() {
  // Handle new video client connections (non-blocking)
  WiFiClient client = videoServer.available();
  if (client) {
    Serial.println("New video client");
    handleMJPEGClient(client);
  }

  // --- I2S Audio reading + sending ---
  static uint8_t i2s_buffer[I2S_BUFFER_SIZE];
  size_t bytes_read = 0;
  esp_err_t ret = i2s_read(I2S_NUM_0, (void*)i2s_buffer, I2S_BUFFER_SIZE, &bytes_read, portMAX_DELAY);
  if (ret == ESP_OK && bytes_read > 0) {
    sendAudioChunk(i2s_buffer, bytes_read);
  }

  // --- GPS reading ---
  while (GPS_Serial.available() > 0) {
    char c = GPS_Serial.read();
    gps.encode(c);
  }
  if (gps.location.isUpdated()) {
    double lat = gps.location.lat();
    double lng = gps.location.lng();
    sendGPSData(lat, lng);
  }
  
  delay(10); // small delay for stability
}
