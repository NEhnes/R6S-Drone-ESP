/*
  Nathan Ehnes
  June 18 2025
  TEJ4M Creative engineering project
*/
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <SPIFFS.h>
#include "esp_camera.h"
#include <Arduino.h>
#include "credentials.h"
#include "config.h"

// motor velocities
float vL = 0;
float vR = 0;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

uint64_t msecs, lastMsecs;
int targetFPS = 10; // was 5
int jpegQuality = 20;
int frameInterval = 1000 / targetFPS;
int cleanupClientInterval = frameInterval / 2;
int frameCounter = 0;

void broadcastCameraFrame();
void driveMotors();
void printData();

void setup() {

  Serial.begin(115200);

  delay(10000); // added bcz serial output is delayed on power up

  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);
  pinMode(PWM_A_PIN, OUTPUT);
  pinMode(PWM_B_PIN, OUTPUT);

  digitalWrite(STBY_PIN, HIGH);

  msecs = lastMsecs = millis();

#pragma region CAMERA_CONFIG
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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_QQVGA;    // 160x120
  config.pixel_format = PIXFORMAT_JPEG;  // For streaming
  config.grab_mode = CAMERA_GRAB_LATEST; // DEFAULT: GRAB_WHEN_EMPTY
  config.fb_location = CAMERA_FB_IN_DRAM;
  config.jpeg_quality = jpegQuality; // 10-63, lower number means higher quality   // DEFAULT: 12
  config.fb_count = 2;               // DEFAULT: 1
#pragma endregion

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("camera init failed with error 0x%x", err); // print error code (hexadecimal)
    while (true); // halt execution
  }
  else Serial.println("camera init successful");

  Serial.println();
  Serial.println("connecting...");
  WiFi.begin(ssid, password);

  // loop while connecting
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println(".");
  }

  // print IP
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.begin();
  Serial.println("server started");

  if (!SPIFFS.begin(true)) { // spiffs error message
    Serial.println("SPIFFS error");
    while(true); // halt execution
  }

  // WebSocket event handler for receiving motor commands
  ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len){
    if (type == WS_EVT_DATA) {
      float* values = (float*)data;
      vL = values[0];
      vR = values[1];
    }
  });

  // Serve index.html from SPIFFS
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {  
    File file = SPIFFS.open("/index.html", "r");
    if (!file) {
      request->send(404, "text/plain", "File not found");
      while(true); // halt execution
    }
    request->send(SPIFFS, "/index.html", "text/html");
    file.close(); 
  });
  server.addHandler(&ws); // add WebSocket
}

//  *********************************************************
//  ***************SETUP END******LOOP START*****************
//  *********************************************************

void loop() {
  msecs = millis();

  if (msecs - lastMsecs > frameInterval){ // hz specified above
    broadcastCameraFrame();

    driveMotors();

    if (++frameCounter % 12 == 0) {
      printData();
      frameCounter = 0;
    }
    lastMsecs = msecs;
  }

  if (msecs - lastMsecs > cleanupClientInterval) { // 24hz
    ws.cleanupClients(); // not necessary to run as often
  }
}

//  *********************************************************
//  ***************CUSTOM METHODS BELOW**********************
//  *********************************************************

void broadcastCameraFrame() {
  camera_fb_t *fb = esp_camera_fb_get(); // fill buffer with new frame

  if (!fb){
    Serial.println("Camera capture failed");
    return;
  }

  // skip sending if buffer full
  if (ws.availableForWriteAll()){
    ws.binaryAll(fb->buf, fb->len);
  }
  else{
    Serial.println("Skipped frame: WebSocket buffer full");
  }

  // return frame buffer
  esp_camera_fb_return(fb);
}

void driveMotors() {
  // this is for tb6612fng, have to write high/lo for direction then write speed to pwm pin
  bool aForward = (vL >= 0); // determine direction
  bool bForward = (vR >= 0);

  // write direction
  Serial.println("Driving motors");
  Serial.println();
  digitalWrite(AIN1_PIN, aForward);
  digitalWrite(AIN2_PIN, !aForward);
  digitalWrite(BIN1_PIN, bForward);
  digitalWrite(BIN2_PIN, !bForward);
  digitalWrite(STBY_PIN, HIGH);

  // write speed
  analogWrite(PWM_A_PIN, abs(vL));
  analogWrite(PWM_B_PIN, abs(vR));
}

void printData() {
  Serial.print("L_Velocity: ");
  Serial.print(vL);
  Serial.print(" /// R_Velocity: ");
  Serial.println(vR);

  Serial.print("Temp: ");
  Serial.println(temperatureRead());
}