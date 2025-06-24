// Libraries
#include <driver/i2s.h>
#include <SPI.h> // Using SPI interface for the SD card
#include <SD.h> // A dedicated library for the SD card
#include "FS.h"

// Defenitions
#define I2S_SAMPLE_RATE           16000 // 16KHz
#define ADC_INPUT                 ADC1_CHANNEL_5 //pin 33
#define I2S_SAMPLE_BITS           I2S_BITS_PER_SAMPLE_16BIT // 16 Bits
#define NUM_OF_CHANNELS               1 // 1 Channel
#define RECORD_TIME               10 // 10 Seconds of recording
#define BUTTON_MIC                    32 // Mic's Button pin number
#define BUTTON_SPEAK                    35 // Speaker's Button pin number

// Global variables
const int chipSelect =            5;
uint32_t wavSize =               NUM_OF_CHANNELS * I2S_SAMPLE_RATE * (I2S_SAMPLE_BITS / 8) * RECORD_TIME;
String file_name =                "/test.wav";

void setup() {
  // Serial monitor
  Serial.begin(115200);
  delay(1000);
  Serial.println("wavSize = " + String(wavSize));

  // Button
  pinMode(BUTTON_MIC, INPUT);
  pinMode(BUTTON_SPEAK, INPUT);
  // Initialize the I2S peripheral
  i2sInit();

  // SD card:
  // see if the card is present and can be initialized:
  while (!SD.begin(chipSelect)) // If there was a faliure
  {
    Serial.println("Card failed, or not present");
    delay(1000);
  }
  Serial.println("card initialized.\nReady to record!");

  delay(1000);
}

void loop() {
  if (digitalRead(BUTTON_MIC)) // Record button pushed = Start recording
  {
    save_audio();
  }
  if (digitalRead(BUTTON_SPEAK)) // Playback button pushed = Start playing
  {
    play_audio();
  }
}

// Mic functions:
void i2sInit()
{
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER  | I2S_MODE_RX |I2S_MODE_ADC_BUILT_IN | I2S_MODE_TX), // Both TX (Speaker) and RX (Mic)
    .sample_rate =  I2S_SAMPLE_RATE,              
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, 
    //.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .channel_format =     I2S_CHANNEL_FMT_ONLY_RIGHT, // Only one channel
    //.communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    //.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .intr_alloc_flags = 0,
    .dma_buf_count = 4,
    .dma_buf_len = 128,
    .use_apll = 1,
  };
  
  static const i2s_pin_config_t pin_config = { // For the Speaker
    .bck_io_num = 27,                                 // The bit clock connectiom, goes to pin 27 of ESP32
    .ws_io_num = 26,                                  // Word select, also known as word select or left right clock
    .data_out_num = 25,                               // Data out from the ESP32, connect to DIN on 38357A
    .data_in_num = I2S_PIN_NO_CHANGE                  // we are not interested in I2S data into the ESP32
};

  
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_sample_rates(I2S_NUM_0, I2S_SAMPLE_RATE);      //set sample rate
  i2s_set_pin(I2S_NUM_0,&pin_config);
  i2s_set_adc_mode(ADC_UNIT_1, ADC_INPUT);
  i2s_adc_enable(I2S_NUM_0);
 
}

void save_audio() { // Recording audio with the MAX9814, and saving to an SD card

  //Serial.println("Opening the file");

  File dataFile = SD.open(file_name, FILE_APPEND);

  //Serial.println("File opened");

  // The 4 high bits are the channel, and the data is inverted
  uint16_t offset = (int)ADC_INPUT * 0x1000 + 0xFFF;

  uint32_t rec_len = 0;
  size_t bytes_read;
  //int buff_len = 2;
  int buff_len = 32;
  uint16_t buffer[buff_len] = {0};
  byte wav_head[44];
  int min_val = 5000, max_val = 0;

  wavHeader(wav_head, wavSize);

  for (int i = 0; i < 44; i++)
  {
    dataFile.write( wav_head[i] );
  }

  Serial.println("Rec starting!");
  int rec_time = millis();
  while (rec_len < wavSize)
  {
    i2s_read(I2S_NUM_0, &buffer, sizeof(buffer), &bytes_read, portMAX_DELAY);
    rec_len += bytes_read;
    for (int i = 0; i < buff_len; i++)
    {
      // Explanation: I want to save 16 bits of information from 
      // every reading.Therfore, I take the uint16_t value, and 
      // compare it (AND gate) first to an 8 bit 1 (0xff = 0000000011111111)
      // Then, I move the 16 bit values to the right by 8 bits 
      // (taking the leftmost values) and compares them to 0xff
      dataFile.write(byte( (offset - buffer[i]) & 0xff));
      dataFile.write(byte( ((offset - buffer[i])>>8) & 0xff));
    }
  }
  rec_time = millis() - rec_time;

  // Read from file and find the maximum value
  dataFile.close();

  
  Serial.println("Rec finished!");
  Serial.println("record length: " + String(rec_len));
  Serial.println("wavSize: " + String(wavSize));
  Serial.println("Expected rec time = " + String(RECORD_TIME));
  Serial.println("Actuall rec time = " + String(rec_time / 1000.0));
}

void play_audio() { // Playing audio from a SD card with the MAX98357

  //Serial.println("Opening the file");

  //File dataFile = SD.open(file_name, FILE_READ);  
  File dataFile = SD.open(file_name);  
  int buff_len = 1024;
  
  uint8_t buffer[buff_len] = {0}; // Reading 1 byte each time, so uint8_t
  size_t BytesWritten;
  dataFile.seek(44); //  The first 44 bytes are wav header, after that there is real audio data
  Serial.println("Start playing!");
  int rec_time = millis();
  while(dataFile.available())
  {
   dataFile.read(buffer,buff_len);
  i2s_write(I2S_NUM_0,buffer,sizeof(buffer),&BytesWritten,portMAX_DELAY); 
  }
  rec_time = millis() - rec_time;
  Serial.println("Finished playing!");
  Serial.println("Played for " + String(rec_time/1000.0) + " seconds");
  dataFile.seek(44); //  The first 44 bytes are wav header, after that there is real audio data
  dataFile.close();
}

// WAV

const int headerSize = 44;

void wavHeader(byte* header, int wavSize) {
  // First 4 are for the name RIFF
  header[0] = 'R';
  header[1] = 'I';
  header[2] = 'F';
  header[3] = 'F';

  // Next is the file size, which can be calculated by the formula below:
  // (I2S_NUM_OF_CHANNELS * I2S_SAMPLE_RATE * (I2S_SAMPLE_BITS / 8) * RECORD_TIME)
  // I multiply by
  unsigned int fileSize = wavSize + headerSize - 8;
  header[4] = (byte)(fileSize & 0xFF);
  header[5] = (byte)((fileSize >> 8) & 0xFF);
  header[6] = (byte)((fileSize >> 16) & 0xFF);
  header[7] = (byte)((fileSize >> 24) & 0xFF);

  // More strings
  header[8] = 'W';
  header[9] = 'A';
  header[10] = 'V';
  header[11] = 'E';
  // fmt, including null
  header[12] = 'f';
  header[13] = 'm';
  header[14] = 't';
  header[15] = ' ';

  // Size of format section (Not inverted)
  header[16] = 0x10;
  header[17] = 0x00;
  header[18] = 0x00;
  header[19] = 0x00;

  // Format (1 = PCM)
  header[20] = 0x01;
  header[21] = 0x00;

  //  Channels (1=mono, 2=stereo)
  //header[22] = 0x01;
  header[22] = 0x01;
  header[23] = 0x00;

  // Sample rate (make sure it matches I2S_SAMPLE_RATE), currently 16KHz
  header[24] = 0x80;
  header[25] = 0x3E;
  header[26] = 0x00;
  header[27] = 0x00;

  // Byte rate  (Sample rate * channels * bits per sample / 8), currently (16,000*1*16/8) = 32,000
  header[28] = 0x00;
  header[29] = 0x7D;
  header[30] = 0x00;
  header[31] = 0x00;

  // Block allign : channels * bits per sample / 8
  header[32] = 0x02;
  header[33] = 0x00;

  // Bits per sample (according to I2S_SAMPLE_BITS)
  header[34] = 0x10;
  header[35] = 0x00;

  // Data name
  header[36] = 'd';
  header[37] = 'a';
  header[38] = 't';
  header[39] = 'a';

  // Size of data
  header[40] = (byte)(wavSize & 0xFF);
  header[41] = (byte)((wavSize >> 8) & 0xFF);
  header[42] = (byte)((wavSize >> 16) & 0xFF);
  header[43] = (byte)((wavSize >> 24) & 0xFF);

}