#!/usr/bin/env python3
import time
import RPi.GPIO as GPIO

RESET_PIN = 12

def reset_stm32():

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
	# waiting for initialisation finished for some time
	time.sleep(1)

if __name__ == '__main__':
	reset_stm32()
	print('Finished')
