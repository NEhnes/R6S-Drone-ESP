/*
  Nathan Ehnes
  June 18 2025
  TEJ4M Creative engineering project
*/

float vL = 0;
float vR = 0;

#pragma region PWM_VARIABLES

// tb6612fng driver, write high/low to IN1/IN2 for direction, then pwm to PWM pin for speed

// left side
#define AIN1_PIN 3
#define AIN2_PIN 4
#define PWM_A 5

// right side
#define BIN1_PIN 2
#define BIN2_PIN 1
#define PWM_B 0

#define STBY_PIN 9


#pragma endregion

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <SPIFFS.h>
#include "esp_camera.h"
#include <Arduino.h>
#include "credentials.h"

#pragma region CAMERA_CONFIG
// Camera configuration for XIAO ESP32S3 Sense
#define PWDN_GPIO_NUM -1  // Power down is not used
#define RESET_GPIO_NUM -1 // Software reset
#define XCLK_GPIO_NUM 10
#define SIOD_GPIO_NUM 40
#define SIOC_GPIO_NUM 39
#define Y9_GPIO_NUM 48
#define Y8_GPIO_NUM 11
#define Y7_GPIO_NUM 12
#define Y6_GPIO_NUM 14
#define Y5_GPIO_NUM 16
#define Y4_GPIO_NUM 18
#define Y3_GPIO_NUM 17
#define Y2_GPIO_NUM 15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM 47
#define PCLK_GPIO_NUM 13
#pragma endregion

#pragma region stream_data

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
String speedString, angleString;

uint64_t msecs, lastMsecs;
int targetFPS = 10; // was 5
int jpegQuality = 10;
int frameInterval = 1000 / targetFPS;
int frameCounter = 0;
#pragma endregion

void BroadcastCameraFrame();
void PrintIP();
void DriveMotors();
void PrintSerialData();

void setup()
{

  delay(10000); // added to clear up a serial output issue

  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);
  pinMode(PWM_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);

  digitalWrite(STBY_PIN, HIGH);

  msecs = lastMsecs = millis();

  // give this shit its own header file later
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
  if (err != ESP_OK) 
  {
    Serial.printf("camera init failed with error 0x%x", err); // print error code (hexadecimal)
    return;
  }
  else Serial.println("camera init successful");

  Serial.begin(115200);
  Serial.println();
  Serial.println("Connecting...");

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.println(".");
  }

  // print IP
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.begin();
  Serial.println("Server started");

  if (!SPIFFS.begin(true)) // spiffs error message
  {
    Serial.println("An error has occurred while mounting SPIFFS");
    return;
  }

#pragma region WEBSOCKET_INIT
  // WebSocket event handler
  ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len){

    if (type == WS_EVT_DATA) {
      float* values = (float*)data;
      vL = values[0];
      vR = values[1];
    }
  });

#pragma endregion

#pragma region SERVE_WEBPAGE
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    
    File file = SPIFFS.open("/index.html", "r");
    if (!file) {
      request->send(404, "text/plain", "File not found");
      return;
    }
    request->send(SPIFFS, "/index.html", "text/html");
    file.close(); });
#pragma endregion

  server.addHandler(&ws); // add WebSocket
}

//  *********************************************************
//  ***************SETUP END******LOOP START*****************
//  *********************************************************

void loop() {
  msecs = millis();

  if (msecs - lastMsecs > frameInterval){ // hz specified above
    BroadcastCameraFrame();
    lastMsecs = msecs;

    DriveMotors();

    frameCounter++;
    if (frameCounter % 12 == 0){
      PrintSerialData();
      frameCounter = 0;
    }
  }

  if (msecs - lastMsecs > frameInterval / 2){ // 24hz
    ws.cleanupClients(); // not necessary to run as often
  }
}

//  *********************************************************
//  ***************CUSTOM METHODS BELOW**********************
//  *********************************************************

void BroadcastCameraFrame()
{
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

void DriveMotors()
{
  // this is for tb6612fng, have to write high/lo for direction then write speed to pwm pin
  bool aForward = (vL >= 0); // determine direction
  bool bForward = (vR >= 0);

  // write direction
  digitalWrite(AIN1_PIN, aForward);
  digitalWrite(AIN2_PIN, !aForward);
  digitalWrite(BIN1_PIN, bForward);
  digitalWrite(BIN2_PIN, !bForward);
  digitalWrite(STBY_PIN, HIGH);

  // write speed
  analogWrite(PWM_A, abs(vL));
  analogWrite(PWM_B, abs(vR));
}

void PrintSerialData(){
  Serial.print("L_Velocity: ");
  Serial.print(vL);
  Serial.print(" /// R_Velocity: ");
  Serial.println(vR);

  Serial.print("Temp: ");
  Serial.println(temperatureRead());
}

void PrintIP()
{
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}