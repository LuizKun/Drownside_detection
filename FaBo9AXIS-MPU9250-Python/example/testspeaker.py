import sys
import RPi.GPIO as GPIO
speaker=11
GPIO.setmode(GPIO.BOARD)
GPIO.setup(speaker,GPIO.OUT, initial = 0)

GPIO.output(speaker,GPIO.LOW)
