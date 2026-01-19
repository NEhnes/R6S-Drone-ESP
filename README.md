# R6S-Drone-ESP

## What is this?

This is a fun personal project: a DIY drone inspired by the ones in Rainbow Six Siege. It's built around a Seeed Studio XIAO ESP32-s3-sense with a camera module, motor drivers, DC motors, and powered by a LiPo battery. The drone sets up a web server to stream video over WebSocket and lets you control it with a joystick from the browser.

## Cool Stuff It Does

- Streams live video from the camera to your browser via WebSocket (low lag!).
- Takes joystick input from the web page to drive the motors – move forward, backward, turn, etc.
- Runs on the tiny XIAO ESP32 with Wi-Fi for wireless control.
- Battery-powered for portability.
- Simple web page for viewing the stream and controlling the drone.

## Hardware Used

- Seeed Studio XIAO ESP32-s3-sense
- OV2640 camera module
- MTB6612FNG motor driver module
- JGA25-370 300rpm 12v DC gearmotors
- 3s, 650mAh LiPo
- MP1584 buck converter
- 5v DC mini fan
- 3-D printed chassis & wheels (ABS/PLA)

## Key Design Choices / Optimizations

Switched HTTPS protocol to a websocket - lower latency and full duplex enables quicker real-time feedback

