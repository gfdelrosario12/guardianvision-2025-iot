#include <SD.h>
#include <SPI.h>
#include "driver/i2s.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <time.h>

// Pin definitions - adjust as needed
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define SD_CS_PIN 5

// WiFi credentials
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";

// I2S config parameters
#define I2S_NUM           I2S_NUM_0
#define SAMPLE_RATE       16000
#define I2S_BUFFER_SIZE   1024

// GPS Serial on UART1
HardwareSerial GPS_Serial(1);

#define SD_CS_PIN 5
#define RECORD_CHUNK_DURATION_MS 30 * 60 * 1000  // 30 minutes in ms

bool trigger_detected = false;
unsigned long trigger_time = 0;
String current_audio_filename = "";
String triggered_audio_filename = "";

unsigned long chunk_start_time = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting setup...");

  // --- Initialize GPS Serial ---
  GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS serial initialized.");

  // --- Initialize SD card with retry ---
  Serial.println("Initializing SD card...");
  while (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card init failed, retrying in 1 sec...");
    delay(1000);
  }
  Serial.println("SD card initialized.");

  // --- Connect to WiFi with retry ---
  Serial.printf("Connecting to WiFi SSID: %s\n", ssid);
  WiFi.begin(ssid, password);
  int wifi_attempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_attempts < 20) {
    delay(500);
    Serial.print(".");
    wifi_attempts++;
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected.");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Failed to connect to WiFi.");
  }

  // --- Configure and wait for NTP time sync ---
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print("Waiting for NTP time sync");
  struct tm timeinfo;
  int wait_time = 0;
  while (!getLocalTime(&timeinfo) && wait_time < 20) {
    Serial.print(".");
    delay(500);
    wait_time++;
  }
  Serial.println();
  if (wait_time < 20) {
    Serial.println("Time synchronized.");
  } else {
    Serial.println("Time sync failed or timed out.");
  }

  // --- Initialize I2S for audio capture ---
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,  // Mono input
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 1024,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = 26,    // Bit Clock Pin
    .ws_io_num = 25,     // Word Select (LRCK) Pin
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = 22    // Data In Pin (MIC data)
  };

  esp_err_t err = i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("Failed installing I2S driver: %d\n", err);
    while(1);
  }
  err = i2s_set_pin(I2S_NUM, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("Failed setting I2S pins: %d\n", err);
    while(1);
  }
  Serial.println("I2S driver initialized.");

  // --- Initialize TensorFlow Lite Micro model ---
  // Add your TFLite model initialization code here
  // Example: tfliteSetup();
  Serial.println("TensorFlow Lite model initialization placeholder.");

  // --- Start first audio chunk ---
  startNewChunk();

  Serial.println("Setup complete.");
}

long getEpochTime() {
  time_t now;
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return 0;
  }
  time(&now);
  return now;
}

void startNewChunk() {
  chunk_start_time = millis();
  current_audio_filename = "/audio_" + String(chunk_start_time) + ".raw";

  Serial.println("Starting new audio chunk: " + current_audio_filename);

  // Open file for writing
  File audioFile = SD.open(current_audio_filename, FILE_WRITE);
  audioFile.close();

  // Reset trigger flag only if we are not continuing after trigger chunk
  if(!trigger_detected) {
    triggered_audio_filename = "";
  }
}

void loop() {
  // 1. Capture audio samples into buffer
  int16_t audio_buffer[16000];  // 1-sec buffer (16kHz)
  captureAudio(audio_buffer);

  // 2. Run ML inference on audio_buffer
  bool triggered_now = runInference(audio_buffer);

  if(triggered_now && !trigger_detected) {
    trigger_detected = true;
    triggered_audio_filename = current_audio_filename;
    trigger_time = millis();

    Serial.println("Trigger word detected! Trigger file: " + triggered_audio_filename);

    // Send trigger event to backend
    sendTriggerEvent(triggered_audio_filename);
  }

  // 3. Append audio_buffer to current chunk file
  appendAudioToFile(current_audio_filename, audio_buffer, sizeof(audio_buffer));

  // 4. Check if 30 mins elapsed in this chunk
  unsigned long elapsed = millis() - chunk_start_time;
  if(elapsed >= RECORD_CHUNK_DURATION_MS) {
    Serial.println("Chunk recording finished: " + current_audio_filename);

    if(trigger_detected) {
      // If this is the chunk after trigger
      if(current_audio_filename != triggered_audio_filename) {
        // Upload triggered_audio_filename + current_audio_filename to S3
        uploadFileToS3(triggered_audio_filename);
        uploadFileToS3(current_audio_filename);

        // Delete local files
        SD.remove(triggered_audio_filename);
        SD.remove(current_audio_filename);

        Serial.println("Uploaded and deleted triggered + next chunk files");

        // Reset trigger flag to start fresh after upload
        trigger_detected = false;
      }
      // else, this chunk was the triggered chunk, start next chunk now
      startNewChunk();

    } else {
      // No trigger detected in this chunk, delete file and start fresh
      SD.remove(current_audio_filename);
      Serial.println("No trigger word in chunk, deleted file: " + current_audio_filename);
      startNewChunk();
    }
  }
}

// Functions you will implement:
void captureAudio(int16_t *buffer) {
  size_t bytesRead = 0;
  size_t bytesToRead = 16000 * sizeof(int16_t);  // 1 sec of 16kHz int16 audio

  // i2s_read reads bytes, so cast buffer to uint8_t*
  esp_err_t result = i2s_read(I2S_NUM_0, (void*)buffer, bytesToRead, &bytesRead, portMAX_DELAY);

  if (result != ESP_OK || bytesRead != bytesToRead) {
    Serial.println("I2S read error or short read");
    memset(buffer, 0, bytesToRead);  // Fill with silence on failure
  }
}

bool runInference(int16_t *buffer) {
  // Run your TensorFlow Lite Micro model here on buffer
  // For demo purposes, we simulate a random trigger detection

  // TODO: Replace with actual ML inference

  static unsigned long lastCheck = 0;
  unsigned long now = millis();

  // Run inference once per second (simulate)
  if (now - lastCheck > 1000) {
    lastCheck = now;
    // Randomly simulate trigger (1% chance)
    if (random(100) < 1) {
      Serial.println("Simulated trigger word detected!");#include <time.h>

long getEpochTime() {
  time_t now;
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return 0;
  }
  time(&now);
  return now;
}
void sendHttpPost(String url, String jsonPayload, const char* tag) {
  HTTPClient http;
  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  int httpResponseCode = http.POST(jsonPayload);

  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.printf("%s event sent, response: %s\n", tag, response.c_str());
  } else {
    Serial.printf("Error sending %s event: %d\n", tag, httpResponseCode);
  }

  http.end();
}

void sendTriggerEvent(String filename) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, cannot send trigger event");
    return;
  }

  double lat, lng;
  String locationStr = "";

  if (getGPSLocation(lat, lng)) {
    locationStr = String(",\"latitude\":") + String(lat, 6) + ",\"longitude\":" + String(lng, 6);
  } else {
    Serial.println("No valid GPS fix available");
  }

  long epochTime = getEpochTime();
  if(epochTime == 0){
    Serial.println("Using millis() fallback for timestamp");
    epochTime = millis() / 1000;  // fallback to millis in seconds
  }

  String jsonPayload = "{\"filename\":\"" + filename + "\",\"timestamp\":" + String(epochTime) + locationStr + "}";

  // URLs — change these to your actual endpoints
  String frontendUrl = "http://your-frontend-server.com/api/trigger";
  String lambdaUrl = "https://your-api-gateway-url.amazonaws.com/prod/trigger";

  sendHttpPost(frontendUrl, jsonPayload, "Frontend");
  sendHttpPost(lambdaUrl, jsonPayload, "Lambda");
}

void uploadFileToS3(String filename) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, cannot upload file");
    return;
  }

  // Step 1: Request presigned URL from your backend
  HTTPClient http;
  String presign_url_endpoint = "http://your-backend-server.com/api/getPresignedUrl?file=" + filename;
  http.begin(presign_url_endpoint);

  int httpCode = http.GET();
  String presignedUrl = "";
  if (httpCode == 200) {
    presignedUrl = http.getString();
    presignedUrl.trim();
    Serial.println("Received presigned URL");
  } else {
    Serial.println("Failed to get presigned URL");
    http.end();
    return;
  }
  http.end();

  // Step 2: Open the file and upload via HTTP PUT to S3
  File file = SD.open(filename, FILE_READ);
  if (!file) {
    Serial.println("Failed to open file for upload");
    return;
  }

  HTTPClient uploadHttp;
  uploadHttp.begin(presignedUrl);
  uploadHttp.addHeader("Content-Type", "application/octet-stream");

  int len = file.size();
  uint8_t* buffer = (uint8_t*)malloc(len);
  if (!buffer) {
    Serial.println("Memory allocation failed");
    file.close();
    return;
  }

  file.read(buffer, len);
  file.close();

  int putCode = uploadHttp.sendRequest("PUT", buffer, len);

  if (putCode == 200 || putCode == 204) {
    Serial.println("File uploaded successfully to S3");
  } else {
    Serial.printf("Failed to upload file: %d\n", putCode);
  }

  free(buffer);
  uploadHttp.end();
}

// Call this periodically to parse GPS data and get latest location
bool getGPSLocation(double &latitude, double &longitude) {
  // Read data from GPS serial and feed it to TinyGPS++ parser
  while (GPS_Serial.available() > 0) {
    char c = GPS_Serial.read();
    gps.encode(c);
  }

  // Check if GPS has a valid fix
  if (gps.location.isValid()) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    return true;
  }
  return false;
}