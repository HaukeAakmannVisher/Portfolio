# Raspberry Pi Runtime (AlphaBot2)

> ⚠️ **Hardware required**  
> This code is written **for the AlphaBot2 platform on a Raspberry Pi** (DC motor driver + TR line sensors + HC-SR04 ultrasonic).  
> **End-to-end execution will NOT work without the AlphaBot2 robot hardware.**  
> (There is no simulator in this repo.)

## What each file does

### `AlphaBot2.py`
- **Role:** Low-level motor driver for the AlphaBot2 (PWM at 500 Hz).
- **Compensation:** Matches right-wheel voltage to the left using calibrated fits  
  - Left:  `V_L(p) = 0.078·p + 0.665`  
  - Right: `V_R(p) = -2.57e−7·p^4 + 7.23e−5·p^3 − 7.58e−3·p^2 + 0.37·p − 0.049`  
  - Binary search → right PWM, then ~3% reduction; **0.5 s torque boost** on right in `forward()`.

### `app.py`
- **Role:** Pi runtime service (Flask). Receives task lists from the laptop (`POST /receive_tasks`), waits for **TR sensor intersections** to execute the next action, and checks **ultrasonic** for obstacles (<5 cm → pause + log).  
- **Sends logs** back to the laptop UI (`/update_log` on your laptop server).

### `intersection_detection.py`
- **Role:** Debug tool for TR sensors. Prints raw values and announces “Intersection detected!” (single threshold).

### `Line_Follow.py`
- **Role:** Minimal intersection logger. Calibrates TR sensor, waits for GPIO7 button, posts a log to the laptop on intersection.

### `QR_scanning.py`
- **Role:** Optional QR reader using a USB camera at `/dev/video0` (pyzbar/OpenCV).  
- **Note:** Camera can work without the robot, but it’s **not** a system simulator.

### `TRSensors.py`
- **Role:** Driver for the TR line sensor array (calibration + `readLine()` used by other scripts).

## Hardware (defaults in code)
- **Right motor (A):** AIN1=13, AIN2=12, ENA/PWMA=6  
- **Left motor  (B):** BIN1=21, BIN2=20, ENB/PWMB=26  
- **PWM:** 500 Hz  
- **Ultrasonic:** TRIG=22, ECHO=27  
- **Camera (optional):** `/dev/video0`

> If your wiring differs, update the pin constants at the top of the files.

## Install (on Raspberry Pi)
```bash
sudo apt-get update
sudo apt-get install -y python3-pip libzbar0
pip3 install RPi.GPIO flask requests opencv-python pyzbar
# If OpenCV is heavy: pip3 install opencv-python-headless
