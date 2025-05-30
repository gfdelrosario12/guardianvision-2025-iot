/*
 * Required Libraries:
 * - WiFi.h
 * - HTTPClient.h
 * - TinyGPS++.h
 * - For camera: ESP32-CAM libraries (esp_camera.h)
 *
 * Note: Adjust pins and settings for your ESP32 setup.
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";
const char* backend = "http://<FLASK_SERVER_IP>:5000";

HardwareSerial gpsSerial(1);
TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);  // RX, TX
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("WiFi connected");
}

void loop() {
  uploadDummyAudio();
  if (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
    if (gps.location.isUpdated()) {
      uploadGPS(gps.location.lat(), gps.location.lng());
    }
  }
  uploadDummyVideo();
  delay(10000);
}

void uploadDummyAudio() {
  uint8_t dummyAudio[8] = {1,2,3,4,5,6,7,8};
  HTTPClient http;
  http.begin(String(backend) + "/upload/audio");
  http.addHeader("Content-Type", "application/octet-stream");
  int code = http.POST(dummyAudio, sizeof(dummyAudio));
  Serial.printf("Audio POST status: %d\n", code);
  http.end();
}

void uploadGPS(float lat, float lng) {
  HTTPClient http;
  http.begin(String(backend) + "/upload/gps");
  http.addHeader("Content-Type", "application/json");
  String json = "{\"lat\": " + String(lat, 6) + ", \"lng\": " + String(lng, 6) + "}";
  int code = http.POST(json);
  Serial.printf("GPS POST status: %d\n", code);
  http.end();
}

void uploadDummyVideo() {
  uint8_t dummyImage[16] = {0xFF, 0xD8, 0xFF, 0xE0}; // JPEG header bytes for example
  HTTPClient http;
  http.begin(String(backend) + "/upload/video");
  http.addHeader("Content-Type", "image/jpeg");
  int code = http.POST(dummyImage, sizeof(dummyImage));
  Serial.printf("Video POST status: %d\n", code);
  http.end();
}
