#include <Arduino.h>
#include "esp_camera.h"
#include <SPI.h>
#include <TFT_eSPI.h>
#include <TFT_eFEX.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include "human_face_detect_msr01.hpp"
#include "human_face_detect_mnp01.hpp"
#include "dl_tool.hpp"
#include <ArduinoJson.h>
#include <base64.h>

#define TWO_STAGE 1
#define RELAY_PIN 0
#define PUSH_BUTTON_PIN 33

TFT_eSPI tft = TFT_eSPI();
TFT_eFEX fex = TFT_eFEX(&tft);

camera_fb_t *fb = NULL;
camera_config_t cameraConfig;

WebSocketsClient webSocket;

const char *ssid = "Redmi Note 10 S";
const char *password = "pusingskripsi";
const char *extraheaders;
boolean isImageOnProcessing = false;
boolean isRegisteringFace = false;
String userId;
JsonDocument doc;

esp_err_t res;

boolean initCamera(camera_config_t config);

void initWiFi(char *ssid, char *password);
void onWebsocketEvent(WStype_t type, uint8_t *payload, size_t length);
void printTextTft(const char* text);

#if TWO_STAGE
    HumanFaceDetectMSR01 s1(0.1F, 0.5F, 10, 0.2F);
    HumanFaceDetectMNP01 s2(0.5F, 0.3F, 5);
#else // ONE_STAGE
    HumanFaceDetectMSR01 s1(0.1F, 0.5F, 10, 0.2F);
#endif

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(PUSH_BUTTON_PIN, INPUT);
  Serial.begin(115200);  

  tft.begin();
  tft.setRotation(0);  // 0 & 2 Portrait. 1 & 3 landscape

  printTextTft("Connecting to the network");
  initWiFi((char *)ssid, (char *)password);
  printTextTft("Connected to the network");
  Serial.println("mac address: " + WiFi.macAddress());
  delay(5000);

  String headersTemp = "x-esp32-id:" + WiFi.macAddress();
  extraheaders = headersTemp.c_str();
  

  webSocket.begin("103.27.206.187", 8081);
  webSocket.setExtraHeaders(extraheaders);
  webSocket.onEvent(onWebsocketEvent);
  webSocket.setReconnectInterval(5000);

  if (webSocket.isConnected()) {
    Serial.println("Connected to the ws server");
    printTextTft("Connected to the ws server");
    delay(1000);
  }

  if (!initCamera(cameraConfig)) {
    Serial.println("Restarting esp....");
    esp_restart();
  }
}

void loop() {
  if (digitalRead(PUSH_BUTTON_PIN) == HIGH) {
    Serial.println("Button pressed");
    digitalWrite(RELAY_PIN, HIGH);
    delay(2000);
    digitalWrite(RELAY_PIN, LOW);
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected");
    initWiFi((char *)ssid, (char *)password);
  }
  res = ESP_OK;
  fb = NULL;
  fb = esp_camera_fb_get();

  size_t out_len;
  int out_width, out_height;
  uint8_t *out_buf;
  size_t _jpg_buf_len;
  uint8_t *_jpg_buf;
  bool converted, faceDetected = false;

  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  int tftWidth = tft.width();
  int tftHeight = tft.height();
  int imgWidth = fb->width;
  int imgHeight = fb->height;

  // Calculate offsets to center the image
  int xOffset = (tftWidth - imgWidth) / 2;
  int yOffset = (tftHeight - imgHeight) / 2;

  // Ensure offsets are within bounds
  if (xOffset < 0) xOffset = 0;
  if (yOffset < 0) yOffset = 0;

  // Ensure image dimensions do not exceed the display dimensions
  if (imgWidth > tftWidth) imgWidth = tftWidth;
  if (imgHeight > tftHeight) imgHeight = tftHeight;

  // Display the image with the calculated offsets
  fex.drawJpg((uint8_t *)fb->buf, fb->len, xOffset, yOffset, imgWidth, imgHeight);
  
  // fex.drawJpg((uint8_t *)fb->buf, fb->len, 0, 0);

  _jpg_buf_len = fb->len;
  _jpg_buf = fb->buf;

  out_len = fb->width * fb->height * 3;
  out_width = fb->width;
  out_height = fb->height;
  out_buf = (uint8_t *)malloc(out_len);
  
  if (!out_buf) {
    Serial.println("out_buf malloc failed");
    free(out_buf);
    res = ESP_FAIL;
  } else {
    converted = fmt2rgb888(fb->buf, fb->len, fb->format, out_buf);
    esp_camera_fb_return(fb);
    fb = NULL;

    if (!converted) {
      log_e("fmt2rgb888 failed");
      free(out_buf);
      res = ESP_FAIL;
    } else {
      #if TWO_STAGE
        std::list<dl::detect::result_t> &candidates = s1.infer((uint8_t *)out_buf, {(int)out_height, (int)out_width, 3});
        std::list<dl::detect::result_t> &results = s2.infer((uint8_t *)out_buf, {(int)out_height, (int)out_width, 3}, candidates);
      #else
        std::list<dl::detect::result_t> &results = s1.infer((uint8_t *)out_buf, {(int)out_height, (int)out_width, 3});
      #endif
      
      if (results.size() > 0) {
        Serial.println("Face detected");
        faceDetected = true;

        dl::detect::result_t prediction = results.front();

        printf("score: %f, box: [%d, %d, %d, %d]\n", prediction.score, prediction.box[0], prediction.box[1], prediction.box[2], prediction.box[3]);

        int x = prediction.box[0];
        int y = prediction.box[1];
        int width = prediction.box[2];
        int height = prediction.box[3];

        if (x < 0) x = 0;
        if (y < 0) y = 0;
        if (x + width > 128) width = 128 - x - 10;
        if (y + height > 160) height = 160 - y - 10;

        fex.drawRect(x, y, width, height, TFT_GREEN);
      }
      converted = fmt2jpg(out_buf, out_len, out_width, out_height, PIXFORMAT_RGB888, 100, &_jpg_buf, &_jpg_buf_len);
      free(out_buf);
      if (!converted) {
        log_e("fmt2jpg failed");
        res = ESP_FAIL;
      }

    }

    if (res == ESP_OK && !isImageOnProcessing) {
      if (webSocket.isConnected() && faceDetected) {
        if (isRegisteringFace) {
          JsonDocument doc;
          doc["destination"] = userId;
          doc["image"] = base64::encode(_jpg_buf, _jpg_buf_len);

          String JsonString;
          serializeJson(doc, JsonString);

          webSocket.sendTXT(JsonString);
          isRegisteringFace = false;
          userId = "";
          Serial.println("image sent to server through webserver for registering face");
        } else {
          webSocket.sendBIN((uint8_t *)_jpg_buf, _jpg_buf_len);
          isImageOnProcessing = true;
          Serial.println("image sent to server through webserver");
        }
      }
    }


    if (fb)
    {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    }
    else if (_jpg_buf)
    {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    else
    {
      log_e("Send frame failed");
    }
  }

  webSocket.loop();
}

void printTextTft(const char* text) {
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(25,55);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  tft.println(text);
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

  config.frame_size = FRAMESIZE_HQVGA;
  config.jpeg_quality = 6;
  config.fb_count = 2;
  config.fb_location = CAMERA_FB_IN_PSRAM; // Frame buffer location

  // Initialize camera with the configured parameters
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
      Serial.println("Camera initialization failed!");
      Serial.printf("Error code: 0x%x \n", err);
      return false;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_HQVGA);
  s->set_hmirror(s, 1);

  return true;
}

void onWebsocketEvent(WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.print("WS Disconnected");
      break;
    case WStype_CONNECTED:
      Serial.print("WS Connected");
      break;
    case WStype_ERROR:
      Serial.print("WS Error");
      break;
    case WStype_TEXT:
      deserializeJson(doc, payload);

      if (doc["message"].as<String>() == "capture")
      {
        userId = doc["user_id"].as<String>();
      } else {
        Serial.println(doc["message"].as<String>());
        Serial.println(doc["open"].as<String>());
        printTextTft(doc["message"].as<String>().c_str());
        delay(1000);

        if (doc["open"].as<boolean>()) {
          Serial.println("open");
          digitalWrite(RELAY_PIN, HIGH);
          delay(2000);
          digitalWrite(RELAY_PIN, LOW);
        } else {
          Serial.println("close");
          digitalWrite(RELAY_PIN, LOW);
        }
        isImageOnProcessing = false;
      }
      break;
    default:
      break;
  }
}

