# R6S-Drone-ESP

## What is this?

This is a fun personal project: a DIY drone inspired by the ones in Rainbow Six Siege. It's built around a Seeed Studio XIAO ESP32-s3-sense with a camera module, motor drivers, DC motors, and powered by a LiPo battery. The drone sets up a web server to stream video over WebSocket and lets you control it with a joystick from the browser.

## Cool Stuff It Does

- Streams live video from the camera to your browser via WebSocket (~10FPS).
- Takes joystick input from the web page to drive the motors.
- Runs on the tiny XIAO ESP32 with Wi-Fi for wireless control.
- Battery-powered for portability.
- Simple web page for viewing the stream and controlling the drone, accesible by IP address

## Hardware Used

- Seeed Studio XIAO ESP32-s3-sense
- OV2640 camera module
- MTB6612FNG motor driver module
- JGA25-370 300rpm 12v DC gearmotors
- 3s, 650mAh LiPo
- MP1584 buck converter
- 5v DC mini fan
- 3-D printed chassis & wheels (ABS/PLA)

## Key Design Choices

### Software/Network:
- Switched HTTPS protocol to a websocket - lower latency and full duplex enables quicker real-time feedback
- Asynchronous websocket - ensures video stream processing doesn't affect control latency
- Binary websocket control messages - computationally efficient parsing
- Frame buffer in DRAM instead of PSRAM - lower latency memory access for real-time streaming
- Dynamic framerate adjustment - accounts for poor connection & overheating risk
- JPEG compression of frames - reduced frame size for intensive wireless transmission
- Separate credentials.h file - password security
- ESP32 in station (STA) mode instead of access point (AP) - less intensive processing

### Electrical/Mechanical:
- Fan for active cooling, heat sink, vent holes & chip temperature monitoring - address heat concerns of the small footprint of the ESP32 board
- Motor driver features flyback protection, thermal shutdown, undervoltage detection, and shoot-through reduction
- 3-D printed chassis

