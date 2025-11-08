#ifndef CONFIG_H
#define CONFIG_H

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

// tb6612fng motor driver pin config
// write high/low to IN1/IN2 for direction, then pwm to PWM pin for speed
#define AIN1_PIN 3
#define AIN2_PIN 4          // left side
#define PWM_A_PIN 5

#define BIN1_PIN 2
#define BIN2_PIN 1          // right side
#define PWM_B_PIN 0

#define STBY_PIN 9

#endif