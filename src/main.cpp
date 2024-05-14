#include <Arduino.h>
#include "esp_camera.h"
#include <SPI.h>
#include <TFT_eSPI.h>
#include <TFT_eFEX.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include "human_face_detect_msr01.hpp"
#include "dl_tool.hpp"

TFT_eSPI tft = TFT_eSPI();
TFT_eFEX fex = TFT_eFEX(&tft);

camera_fb_t *fb = NULL;
camera_config_t cameraConfig;

WebSocketsClient webSocket;

const char *ssid = "Redmi Note 10 S";
const char *password = "pusingskripsi";
boolean isImageProcessing = true;

boolean initCamera(camera_config_t config);

void initWiFi(char *ssid, char *password);
void onWebsocketEvent(WStype_t type, uint8_t *payload, size_t length);

HumanFaceDetectMSR01 humanFaceDetectMSR01(0.1f, 0.5f, 10, 0.2f);

void setup() {
  pinMode(33, OUTPUT);
  Serial.begin(115200);

  tft.begin();
  tft.setRotation(0);  // 0 & 2 Portrait. 1 & 3 landscape
  initWiFi((char *)ssid, (char *)password);
  delay(5000);

  // webSocket.setAuthorization("uwais", "uw415_4Lqarn1");
  webSocket.begin("20.6.129.71", 5096);
  webSocket.onEvent(onWebsocketEvent);
  webSocket.setReconnectInterval(5000);

  Serial.println(webSocket.isConnected());

  if (!initCamera(cameraConfig)) {
    Serial.println("Restarting esp....");
    esp_restart();
  }
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected");
    initWiFi((char *)ssid, (char *)password);
  }

  fb = NULL;
  fb = esp_camera_fb_get();

  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  // size_t out_len;
  // int out_width, out_height;
  // uint8_t *out_buf;

  // out_len = fb->width * fb->height * 3;
  // out_width = fb->width;
  // out_height = fb->height;
  // out_buf = (uint8_t *)malloc(out_len);

  // std::list<dl::detect::result_t> &results = humanFaceDetectMSR01.infer((uint8_t *)fb->buf, {(int)fb->width, (int)fb->height, 3});
  // Serial.println(results.size());

  // if (!out_buf) {
  //     log_e("out_buf malloc failed");
  //   }


  // std::list<dl::detect::result_t> &results = humanFaceDetectMSR01.infer((uint8_t *)out_buf, {(int)out_height, (int)out_width, 3});
  
  // if (results.size() > 0) {
  //   Serial.println("Face detected" + results.size());
  // }

  // free(out_buf);

  // for (std::list<dl::detect::result_t>::iterator prediction = results.begin(); prediction != results.end(); prediction++) {
  //   Serial.printf("Prediction: Score { %f }, box [ %f, %f, %f, %f ]\n", prediction->score, prediction->box[0], prediction->box[1], prediction->box[2], prediction->box[3]);
  // }

  // sending image to websocket
  if (webSocket.isConnected() && !isImageProcessing) {
    webSocket.sendBIN((uint8_t *)fb->buf, fb->len);
    isImageProcessing = true;
    digitalWrite(33, HIGH);
    Serial.println("image sent to server through websocket");
  }

  fex.drawJpg((uint8_t *)fb->buf, fb->len, 0, 0);
  esp_camera_fb_return(fb);

  webSocket.loop();
}

void initWiFi(char *ssid, char *password) {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.printf("\nConnected to WiFi\n");
}

boolean initCamera(camera_config_t config) {
  // Configure GPIO pin assignments for camera interface
  config.pin_pwdn = -1;     // Power-down pin (not used)
  config.pin_reset = -1;    // Reset pin (not used)
  config.pin_xclk = 21;     // External clock pin (CAM_XCLK)
  config.pin_sscb_sda = 26; // SCCB data pin (CAM_SIOD)
  config.pin_sscb_scl = 27; // SCCB clock pin (CAM_SIOC)
  config.pin_d7 = 35;       // Data line D7 (CAM_Y9)
  config.pin_d6 = 34;       // Data line D6 (CAM_Y8)
  config.pin_d5 = 39;       // Data line D5 (CAM_Y7)
  config.pin_d4 = 36;       // Data line D4 (CAM_Y6)
  config.pin_d3 = 19;       // Data line D3 (CAM_Y5)
  config.pin_d2 = 18;       // Data line D2 (CAM_Y4)
  config.pin_d1 = 5;        // Data line D1 (CAM_Y3)
  config.pin_d0 = 4;        // Data line D0 (CAM_Y2)

  config.pin_vsync = 25;    // VSYNC pin (CAM_VSYNC)
  config.pin_href = 23;     // HREF pin (CAM_HREF)
  config.pin_pclk = 22;     // Pixel clock pin (CAM_PCLK)

  config.xclk_freq_hz = 20000000; // External clock frequency (20 MHz)
  config.pixel_format = PIXFORMAT_JPEG; // Pixel format (JPEG)
  // config.pixel_format = PIXFORMAT_RGB565; // Pixel format (JPEG)

  config.frame_size = FRAMESIZE_240X240;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  config.fb_location = CAMERA_FB_IN_PSRAM; // Frame buffer location

  // Initialize camera with the configured parameters
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
      Serial.println("Camera initialization failed!");
      Serial.printf("Error code: 0x%x \n", err);
      return false;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_240X240);

  return true;
}

void onWebsocketEvent(WStype_t type, uint8_t *payload, size_t length) {
  String message;

  switch (type) {
    case WStype_DISCONNECTED:
      Serial.print("WS Disconnected");
      break;
    case WStype_CONNECTED:
      Serial.print("WS Connected");
      webSocket.sendTXT("Hello from ESP32");
      break;
    case WStype_ERROR:
      Serial.print("WS Error");
      break;
    case WStype_TEXT:
      message = String((char *)payload);
      if (message.equals("IMAGE_PROCESSED")) {
        Serial.println("Image processed");
        isImageProcessing = false;
        digitalWrite(33, LOW);
        // delay(5000);
      }
      break;
    default:
      break;
  }
}