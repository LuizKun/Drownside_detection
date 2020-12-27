
import sys
import time
import RPi.GPIO as GPIO

def pin_int():
	global speaker,led_system,led_status,chair_mode1,chair_mode2
	speaker=11
	led_system=33
	led_status=13
	chair_mode1=29
	chair_mode2=31
	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(speaker,GPIO.OUT, initial = 0)
	GPIO.setup(led_system,GPIO.OUT, initial = 0)
	GPIO.setup(led_status,GPIO.OUT, initial = 0)
	GPIO.setup(chair_mode1,GPIO.OUT, initial = 0)
	GPIO.setup(chair_mode2,GPIO.OUT, initial = 0)
def led_blinking(count,pulse):
	for i in range(count):
		time.sleep(pulse)
		GPIO.output(led_system,GPIO.HIGH)
		time.sleep(pulse)
		GPIO.output(led_system,GPIO.LOW)
def status_blinking(count,pulse):
        for i in range(count):
                time.sleep(pulse)
                GPIO.output(led_status,GPIO.LOW)
                time.sleep(pulse)
                GPIO.output(led_status,GPIO.HIGH)
def status_blinking(count,pulse):
        for i in range(count):
                time.sleep(pulse)
                GPIO.output(led_status,GPIO.LOW)
                time.sleep(pulse)
                GPIO.output(led_status,GPIO.HIGH)
def speaker_alert(count,pulse):
        for i in range(count):
                time.sleep(pulse)
                GPIO.output(speaker,GPIO.HIGH)
                time.sleep(pulse+0.3)
                GPIO.output(speaker,GPIO.LOW)

pin_int()
GPIO.output(led_status,GPIO.LOW)
GPIO.output(speaker,GPIO.HIGH)
GPIO.output(led_system,GPIO.HIGH)
time.sleep(3)
GPIO.output(speaker,GPIO.LOW)
GPIO.output(29,GPIO.LOW)
GPIO.output(31,GPIO.LOW)
while(1):
	#speaker_alert(10,0.2)
	#led_blinking(50,0.5)
	#status_blinking(50,0.5)
	time.sleep(2)
GPIO.output(led_system,GPIO.LOW)
time.sleep(1)
