/*
  Nathan Ehnes
  June 18 2025
  TEJ4M Creative engineering project

  **I did not write code for websocket event listeners, just the details within
  **Camera config is straight from an example repo, aside from image resolution/quality
  **Certain other minor details were beyond my goals for this project
*/

float angle = 0;
float speed = 0;

#pragma region PWM_VARIABLES

#define AIN1_PIN 3
#define AIN2_PIN 4
#define PWM_A 5
bool aForward = true;

#define BIN1_PIN 2
#define BIN2_PIN 1
#define PWM_B 0
bool bForward = true;

#define STBY_PIN 9

int lMotorSpeed = 50;
int rMotorSpeed = 50;

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
void InputToPWM();
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
      angle = values[0];
      speed = values[1];
    }
    

    
      // recv and build msg string
      // String msg = "";
      // for (size_t i = 0; i < len; i++) {
      //   msg += (char)data[i];
      // }
      // Serial.println("Received: " + msg);

      /* PARSE JOYSTICK DATA */
      // if (msg.substring(0, 6) != "AT REST"){
      //   int speedStartIndex = msg.indexOf("SPEED: ") + 7;
      //   speedString = msg.substring(speedStartIndex, speedStartIndex + 3); // grab first 3 char (incl '.')
      //   int angleStartIndex = msg.indexOf("ANGLE: ") + 7;
      //   angleString = msg.substring(angleStartIndex, angleStartIndex + 3);
      //   if (speedString.substring(2, 1) == "."){
      //     speedString.remove(2); //3 digit numbers
      //   } else if (speedString.substring(1, 1) == "."){
      //     speedString.remove(1, 2); //single digit number
      //   }
      //   Serial.print("SPEED: ");
      //   Serial.println(speedString);
      // } else {
      //   speedString = "0";
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
    
    // int speedInt = speedString.toInt();
    // double angleInt = angleString.toDouble();
    
    InputToPWM();
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

void PrintIP()
{
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void DriveMotors()
{ // ADDED FROM OTHER PROJ

  // write direction
  digitalWrite(AIN1_PIN, aForward);
  digitalWrite(AIN2_PIN, !aForward);
  digitalWrite(BIN1_PIN, bForward);
  digitalWrite(BIN2_PIN, !bForward);
  digitalWrite(STBY_PIN, HIGH);

  // write speed
  analogWrite(PWM_A, abs(lMotorSpeed));
  analogWrite(PWM_B, abs(rMotorSpeed));
}

void InputToPWM(){

  // use simplified taylor series instead of sin/cos to improve performance if needed
  // don't need double accuracy. ints are fine
  // int speed = _speed;
  speed = constrain(speed, 0, 100);
  // int angle = (int) _angle;

  Serial.print("Speed: ");
  Serial.print(speed);
  Serial.print(" ||| Angle: ");
  Serial.println(angle);

  // determine quadrant for later
  int quadrant;
  if (angle < 90){
    quadrant = 1;
  } else if (angle < 180) {
    quadrant = 4;
  } else if (angle < 270) {
    quadrant = 3;
  } else {
    quadrant = 2;
  }

  float angleRadians = angle * PI / 180.0;

  // calculate components
  int yComponent = (int) speed * sin(angleRadians);
  int xComponent = (int) speed * cos(angleRadians);

  // +/- direction
  if (quadrant >= 3){
    yComponent = -yComponent;
  }
  if (quadrant == 2 || quadrant == 3){
    xComponent = -xComponent;
  }

  // sum left/right speeds
  lMotorSpeed = yComponent + xComponent/2;
  rMotorSpeed = yComponent - xComponent/2;

  aForward = (lMotorSpeed >= 0);
  bForward = (rMotorSpeed >= 0);


  // clamp and convert values to pwm range
  if (lMotorSpeed > 100) lMotorSpeed = 100;
  if (lMotorSpeed < 0) lMotorSpeed = 0;
  lMotorSpeed = map(lMotorSpeed, 0, 100, 0, 255);

  if (rMotorSpeed > 100) rMotorSpeed = 100;
  if (rMotorSpeed < 0) rMotorSpeed = 0;
  rMotorSpeed = map(rMotorSpeed, 0, 100, 0, 255);
}

void PrintSerialData(){
  Serial.print("L_Forward: ");
  Serial.print(aForward);
  Serial.print(" /// R_Forward: ");
  Serial.println(bForward);

  Serial.print("LEFT PWM OUTPUT: ");
  Serial.println(lMotorSpeed);
  Serial.print("RIGHT PWM OUTPUT: ");
  Serial.println(rMotorSpeed);

  Serial.print("Temp: ");
  Serial.println(temperatureRead());
}