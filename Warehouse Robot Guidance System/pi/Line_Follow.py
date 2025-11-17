#!/usr/bin/python
# -*- coding:utf-8 -*-
import RPi.GPIO as GPIO
from TRSensors import TRSensor
import time
import requests

Button = 7

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(Button, GPIO.IN, GPIO.PUD_UP)

TR = TRSensor()

# Calibrate the TRSensor
print("Starting calibration...")
for i in range(0, 100):
    TR.calibrate()

print("Calibration complete")
print(TR.calibratedMin)
print(TR.calibratedMax)

# Wait for the button to be pressed to start the line detection
while GPIO.input(Button) != 0:
    position, Sensors = TR.readLine()
    print(position, Sensors)
    time.sleep(0.05)

print("Ready to detect intersections")

# Main loop to detect intersections
while True:
    try:
        position, Sensors = TR.readLine()

        # Condition for detecting an intersection
        if Sensors[0] > 900 and Sensors[1] > 900 and Sensors[2] > 900 and Sensors[3] > 900 and Sensors[4] > 900:
            print("Intersection detected")

            # Send update to the web interface (update this with the actual IP address/URL of the Flask server)
            try:
                response = requests.post('http://127.0.0.1:5000/update_log', json={"message": "Intersection detected"})
                print(f"Sent to web interface: {response.status_code}")
            except requests.exceptions.RequestException as e:
                print(f"Error sending to web interface: {e}")

        time.sleep(0.05)

    except KeyboardInterrupt:
        print("Stopping")
        break
