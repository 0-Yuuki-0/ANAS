import RPi.GPIO as GPIO
import time
from time import sleep

GPIO.setmode(GPIO.BCM)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(16, GPIO.OUT)
GPIO.setup(26, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)
GPIO.output(12, 1)
GPIO.output(16, 1)
GPIO.output(26, 1)
GPIO.output(21, 1)

def pitchup():
    GPIO.output(16, 0)
    sleep(0.1)
    GPIO.output(16, 1)

def pitchdown():
    GPIO.output(26, 0)
    sleep(0.1)
    GPIO.output(26, 1)

def pitchstop():
    GPIO.output(12, 0)
    sleep(0.1)
    GPIO.output(12, 1)

def fire():
    GPIO.output(21, 0)
    sleep(0.1)
    GPIO.output(21, 1)

try:
    while True:
        var = input()
        if var == "d":
            pitchup()
        if var == "a":
            pitchdown()
        if var == "s":
            pitchstop()
        if var == "f":
            fire()

except KeyboardInterrupt:          # trap a CTRL+C keyboard interrupt  
    GPIO.cleanup()

finally:
    GPIO.cleanup()
