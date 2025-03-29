#define CAMERA_MODEL_XIAO_ESP32S3
#include <driver/i2s.h>  // Use ESP-IDF I2S driver
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include "esp_camera.h"
#include "camera_pins.h"
#include "mulaw.h"

//
// BLE
//

static BLEUUID serviceUUID("19B10000-E8F2-537E-4F6C-D104768A1214");
static BLEUUID audioCharUUID("19B10001-E8F2-537E-4F6C-D104768A1214");
static BLEUUID audioCodecUUID("19B10002-E8F2-537E-4F6C-D104768A1214");
static BLEUUID photoCharUUID("19B10005-E8F2-537E-4F6C-D104768A1214");

BLECharacteristic *audio;
BLECharacteristic *photo;
bool connected = false;

class ServerHandler : public BLEServerCallbacks {
  void onConnect(BLEServer *server) {
    connected = true;
    Serial.println("Connected");
  }

  void onDisconnect(BLEServer *server) {
    connected = false;
    Serial.println("Disconnected");
    BLEDevice::startAdvertising();
  }
};

class MessageHandler : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic, esp_ble_gatts_cb_param_t *param) {
    // Currently unused
  }
};

volatile bool photo_ack_received = false;
class PhotoCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic, esp_ble_gatts_cb_param_t *param) {
    // Serial.println("ACK received from client");
    // Set a global flag to proceed
    photo_ack_received = true;
  }
};

void configure_ble() {
  BLEDevice::init("OpenGlass");
  BLEServer *server = BLEDevice::createServer();
  BLEService *service = server->createService(serviceUUID);

  // Audio service
  audio = service->createCharacteristic(
    audioCharUUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_INDICATE
  );
  BLE2902 *ccc = new BLE2902();
  ccc->setNotifications(true);
  audio->addDescriptor(ccc);

  // Photo service
  photo = service->createCharacteristic(
    photoCharUUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
  );
  ccc = new BLE2902();
  ccc->setNotifications(true);
  photo->addDescriptor(ccc);
  // set callback
  photo->setCallbacks(new PhotoCallback());

  // Codec service
  BLECharacteristic *codec = service->createCharacteristic(
    audioCodecUUID,
    BLECharacteristic::PROPERTY_READ
  );
  uint8_t codecId = 11; // MuLaw 8kHz
  codec->setValue(&codecId, 1);

  // Service
  server->setCallbacks(new ServerHandler());
  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(service->getUUID());
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x0);
  advertising->setMinPreferred(0x1F);
  BLEDevice::startAdvertising();
}


camera_fb_t *fb = NULL;

bool take_photo() {
  if (fb != NULL) {
    Serial.println("Release FB");
    esp_camera_fb_return(fb);
    fb = NULL;
  }

  Serial.println("Taking photo...");
  fb = esp_camera_fb_get();
  Serial.println("Taking photo done...");
  if (!fb) {
    Serial.println("Failed to get camera frame buffer");
    return false;
  }
  Serial.printf("fb pointer address: %p\n", fb);
  Serial.printf("Photo taken, size: %d bytes\n", fb->len);
  Serial.printf("fb read\n");
  return true;
}

//
// Microphone
//

#define VOLUME_GAIN 2
#define SAMPLE_RATE 8000  // Fixed for MuLaw codec
#define SAMPLE_BITS 16

static size_t recording_buffer_size = 400;
static size_t compressed_buffer_size = 400 + 3; /* header */
static uint8_t *s_recording_buffer = nullptr;
static uint8_t *s_compressed_frame = nullptr;
static uint8_t *s_compressed_frame_2 = nullptr;

// void configure_microphone() {
//   // Configure I2S for PDM microphone
//   i2s_config_t i2s_config = {
//     .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),  // Master, receive, PDM mode
//     .sample_rate = SAMPLE_RATE,  // 8kHz for MuLaw
//     .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
//     .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,  // Mono PDM
//     .communication_format = I2S_COMM_FORMAT_STAND_I2S,
//     .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
//     .dma_buf_count = 8,
//     .dma_buf_len = 64,
//     .use_apll = false,
//     .tx_desc_auto_clear = false,
//     .fixed_mclk = 0
//   };

//   i2s_pin_config_t pin_config = {
//     .bck_io_num = 42,    // BCLK
//     .ws_io_num = 41,     // WS (Word Select)
//     .data_out_num = I2S_PIN_NO_CHANGE,
//     .data_in_num = I2S_PIN_NO_CHANGE
//   };

//   // Install and start I2S driver
//   esp_err_t err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
//   if (err != ESP_OK) {
//     Serial.printf("Failed to install I2S driver: %d\n", err);
//     while (1);
//   }

//   err = i2s_set_pin(I2S_NUM_0, &pin_config);
//   if (err != ESP_OK) {
//     Serial.printf("Failed to set I2S pins: %d\n", err);
//     while (1);
//   }

//   // Allocate buffers
//   s_recording_buffer = (uint8_t *)ps_calloc(recording_buffer_size, sizeof(uint8_t));
//   s_compressed_frame = (uint8_t *)ps_calloc(compressed_buffer_size, sizeof(uint8_t));
//   s_compressed_frame_2 = (uint8_t *)ps_calloc(compressed_buffer_size, sizeof(uint8_t));
// }

// size_t read_microphone() {
//   size_t bytes_recorded = 0;
//   esp_err_t err = i2s_read(I2S_NUM_0, s_recording_buffer, recording_buffer_size, &bytes_recorded, portMAX_DELAY);
//   if (err != ESP_OK) {
//     Serial.printf("I2S read error: %d\n", err);
//   }
//   return bytes_recorded;
// }

//
// Camera
//

void configure_camera() {
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
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  config.fb_count = 1;

  config.jpeg_quality = 10;
  config.frame_size = FRAMESIZE_SVGA;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  s_compressed_frame_2 = (uint8_t *)ps_calloc(compressed_buffer_size, sizeof(uint8_t));

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    while (1);  // Halt to catch init failure
  }
  Serial.println("Camera initialized successfully");
}

//
// Main
//

void setup() {
  Serial.begin(921600);
  Serial.println("Setup");
  Serial.println("Starting BLE...");
  configure_ble();
  Serial.println("Starting Microphone...");
  // configure_microphone();
  Serial.println("Starting Camera...");
  configure_camera();
  Serial.println("OK");
  pinMode(LED_BUILTIN, OUTPUT);
}

uint16_t frame_count = 0;
unsigned long lastCaptureTime = 0;
size_t sent_photo_bytes = 0;
size_t sent_photo_frames = 0;
bool need_send_photo = false;

void loop() {
  
  // Serial.println("Starting Loop...");
  // Read from mic
  // size_t bytes_recorded = read_microphone();
  // // Push to BLE
  // if (bytes_recorded > 0 && connected) {
  //   size_t out_buffer_size = bytes_recorded / 2 + 3;
  //   for (size_t i = 0; i < bytes_recorded; i += 2) {
  //     int16_t sample = ((s_recording_buffer[i + 1] << 8) | s_recording_buffer[i]) << VOLUME_GAIN;
  //     s_compressed_frame[i / 2 + 3] = linear2ulaw(sample);
  //   }
  //   s_compressed_frame[0] = frame_count & 0xFF;
  //   s_compressed_frame[1] = (frame_count >> 8) & 0xFF;
  //   s_compressed_frame[2] = 0;
  //   audio->setValue(s_compressed_frame, out_buffer_size);
  //   audio->notify();
  //   frame_count++;
  // }

  // Take a photo
  unsigned long now = millis();
  // Serial.printf("time passed %d",now);

  // if ((now - lastCaptureTime) % 500 == 0) {
  //   Serial.println("500 time passed");
  // }
  if ((now - lastCaptureTime) >= 5000 && !need_send_photo && connected) {
  // if (connected) {
    if (take_photo()) {
      need_send_photo = true;
      sent_photo_bytes = 0;
      sent_photo_frames = 0;
      lastCaptureTime = now;
    }
  }

  // Push to BLE
  if (need_send_photo) {
    size_t remaining = fb->len - sent_photo_bytes;
    if (remaining > 0) {
      // start header
      // s_compressed_frame_2[0] = 0xAA;
      // s_compressed_frame_2[1] = 0xAA;
      // Populate buffer
      s_compressed_frame_2[0] = sent_photo_frames & 0xFF;
      s_compressed_frame_2[1] = (sent_photo_frames >> 8) & 0xFF;
      size_t bytes_to_copy = remaining;
      if (bytes_to_copy > 200) {
        bytes_to_copy = 200;
      }
      // Serial.printf("Sending frame %d, %d bytes, total sent: %d/%d\n", 
      //         sent_photo_frames, bytes_to_copy, sent_photo_bytes + bytes_to_copy, fb->len);
      memcpy(&s_compressed_frame_2[2], &fb->buf[sent_photo_bytes], bytes_to_copy);

      // // 打印帧数据
      // Serial.printf("Frame %d: ", sent_photo_frames);
      // for (size_t i = 0; i < bytes_to_copy + 2; i++) {
      //   Serial.printf("%02X ", s_compressed_frame_2[i]);
      // }
      // Serial.println();

      // Push to BLE
      photo_ack_received = false;
      photo->setValue(s_compressed_frame_2, bytes_to_copy + 2);
      photo->notify(); 
      unsigned long timeout = millis();

      // ACK check 
      while (!photo_ack_received && (millis() - timeout) < 500) {
        delay(10);  // Wait up to 1s for ACK
      }
      // if (!photo_ack_received) {
      //   Serial.println("No ACK, next byte");
      //   // }
      //   photo_ack_received = false;  // Reset for next frame
      //   sent_photo_bytes += bytes_to_copy;
      //   sent_photo_frames++;
      // } else {
      //   photo_ack_received = false;  // Reset for next frame
      //   sent_photo_bytes += bytes_to_copy;
      //   sent_photo_frames++;
      // }
      sent_photo_bytes += bytes_to_copy;
      sent_photo_frames++;
    } else {
      // End flag
      s_compressed_frame_2[0] = 0xFF;
      s_compressed_frame_2[1] = 0xFF;
      photo->setValue(s_compressed_frame_2, 2);
      photo->notify(); 

      Serial.println("Photo sent");
      need_send_photo = false;
    }
  }

  // Delay
  // digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  // delay(1000);                      // wait for a second
  // digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  // delay(1000); 
  // digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  // delay(1000);                      // wait for a second
  // digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  // delay(1000); 
  // digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  // delay(1000);                      // wait for a second
  // digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  // delay(1000); 
  // digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  // delay(1000);                      // wait for a second
  // digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  // delay(1000); 
  // delay(20); 

}