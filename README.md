# R6S-Drone-ESP

## What is this?

This is a fun personal project: a DIY drone inspired by the ones in Rainbow Six Siege. It's built around a Seeed Studio XIAO ESP32 with a camera, motor drivers, DC motors, and powered by a LiPo battery. The drone sets up a web server to stream video over WebSocket and lets you control it with a joystick from the browser. Great for tinkering, learning ESP32 stuff, or just messing around with remote-controlled gadgets.

## Cool Stuff It Does

- Streams live video from the camera to your browser via WebSocket (low lag!).
- Takes joystick input from the web page to drive the motors – move forward, backward, turn, etc.
- Runs on the tiny XIAO ESP32 with Wi-Fi for wireless control.
- Battery-powered for portability.
- Simple web page for viewing the stream and controlling the drone.

## Hardware I Used

- Seeed Studio XIAO ESP32
- Camera module (like OV2640 that works with ESP32)
- Motor driver board (I used something like L298N)
- A couple DC motors for wheels/tracks
- LiPo battery to power it all
- Some kind of chassis (I 3D-printed a simple one, but anything works)

## Notes

This is just a demo I threw together for fun – not polished or production-ready. If it breaks, hack it! Feel free to fork and improve.

No license or anything formal – use at your own risk.
