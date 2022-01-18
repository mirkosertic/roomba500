#!/usr/bin/env python3

import time
import RPi.GPIO as GPIO


GPIO.setmode(GPIO.BOARD)   # BCM for GPIO numbering
GPIO.setup(11, GPIO.OUT) # Make pin 17 (which is hooked up to the BRC pin) an output

# Pulse the BRC pin at a low duty cycle to keep Roomba awake.
while True:
    print("HIGH")
    GPIO.output(11, GPIO.HIGH)
    time.sleep(1)
    print("LOW")
    GPIO.output(11, GPIO.LOW)
    time.sleep(1)
    print("LOW")
    GPIO.output(11, GPIO.HIGH)
    time.sleep(1)
