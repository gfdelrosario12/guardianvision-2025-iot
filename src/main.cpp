#include <Arduino.h>

// Prevent Arduino macro conflict with TFLite
#ifdef DEFAULT
#undef DEFAULT
#endif

#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"
#include <driver/i2s.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

#include "word_model_tflite.h" // Must be correctly generated from .tflite as C array

// TensorFlow Lite Micro
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

// ========== PIN DEFINITIONS ==========
#define I2S_WS  25
#define I2S_SD  32
#define I2S_SCK 33
#define GPS_RX 16
#define GPS_TX 17

// ========== GLOBALS ==========
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;

// TensorFlow Lite globals
constexpr int kTensorArenaSize = 100 * 1024; // 100KB
uint8_t* tensor_arena = nullptr;

tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input = nullptr;
TfLiteTensor* output = nullptr;

// ========== CAMERA INIT ==========
void initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = 5;
  config.pin_d1 = 18;
  config.pin_d2 = 19;
  config.pin_d3 = 21;
  config.pin_d4 = 36;
  config.pin_d5 = 39;
  config.pin_d6 = 34;
  config.pin_d7 = 35;
  config.pin_xclk = 0;
  config.pin_pclk = 22;
  config.pin_vsync = 25;
  config.pin_href = 23;
  config.pin_sscb_sda = 26;
  config.pin_sscb_scl = 27;
  config.pin_pwdn = 32;
  config.pin_reset = -1;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 10;
  config.fb_count = 1;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed!");
  }
}

// ========== I2S INIT ==========
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

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
}

// ========== SD CARD INIT ==========
void initSDCard() {
  if (!SD_MMC.begin()) {
    Serial.println("SD Card Mount Failed");
    return;
  }
  if (SD_MMC.cardType() == CARD_NONE) {
    Serial.println("No SD card detected");
    return;
  }
  Serial.println("SD Card ready");
}

// ========== GPS INIT ==========
void initGPS() {
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
}

// ========== TFLITE INIT ==========
void setupModel() {
  tensor_arena = (uint8_t*)ps_malloc(kTensorArenaSize);
  if (!tensor_arena) {
    Serial.println("Failed to allocate tensor arena!");
    while (true);
  }

  const tflite::Model* model = tflite::GetModel(word_model_tflite);
  static tflite::AllOpsResolver resolver;

  static tflite::MicroInterpreter staticInterpreter(
    model, resolver, tensor_arena, kTensorArenaSize, nullptr
  );

  interpreter = &staticInterpreter;
  interpreter->AllocateTensors();

  input = interpreter->input(0);
  output = interpreter->output(0);

  Serial.println("Model setup complete");
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(1000);

  initCamera();
  initI2SMic();
  initSDCard();
  initGPS();
  setupModel();

  Serial.println("Setup complete.");
}

// ========== LOOP ==========
void loop() {
  size_t bytes_read = 0;
  int16_t sample_buffer[512];

  i2s_read(I2S_NUM_0, sample_buffer, sizeof(sample_buffer), &bytes_read, portMAX_DELAY);

  if (bytes_read > 0) {
    memcpy(input->data.i16, sample_buffer, bytes_read);
    TfLiteStatus status = interpreter->Invoke();

    if (status != kTfLiteOk) {
      Serial.println("Inference failed");
      return;
    }

    int prediction = output->data.i16[0]; // Change based on model output
    Serial.printf("Prediction: %d\n", prediction);

    if (prediction > 0) {
      while (gpsSerial.available()) gps.encode(gpsSerial.read());

      if (gps.location.isValid()) {
        Serial.printf("GPS: %f, %f\n", gps.location.lat(), gps.location.lng());

        File log = SD_MMC.open("/log.txt", FILE_APPEND);
        if (log) {
          log.printf("Keyword detected at: %f,%f\n", gps.location.lat(), gps.location.lng());
          log.close();
        }

        camera_fb_t* fb = esp_camera_fb_get();
        if (fb) {
          File pic = SD_MMC.open("/photo.jpg", FILE_WRITE);
          if (pic) {
            pic.write(fb->buf, fb->len);
            pic.close();
            Serial.println("Saved image.");
          }
          esp_camera_fb_return(fb);
        }
      }
    }
  }

  delay(2000);
}
