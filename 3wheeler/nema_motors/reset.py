#!/usr/bin/env python3
import time
import RPi.GPIO as GPIO

RESET_PIN = 12

# for GPIO numbering, choose BCM
#GPIO.setmode(GPIO.BCM)

# or, for pin numbering, choose BOARD  
GPIO.setmode(GPIO.BOARD)

GPIO.setwarnings(False)
#reset
GPIO.setup(RESET_PIN, GPIO.OUT)

# flash firmware
GPIO.output(RESET_PIN, 0)
time.sleep(0.2)
GPIO.output(RESET_PIN, 1)

print('Finished')
