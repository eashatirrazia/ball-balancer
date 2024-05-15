import RPi.GPIO as GPIO
import time

DIR = 8
STEP = 10
stepsPerRevolution = 3200

def delay_ms(n):
    time.sleep(n*1e-6)

def high(pin, log=False):
    GPIO.output(pin, GPIO.HIGH)
    if log:
        print("high")

def low(pin, log=False):
    GPIO.output(pin, GPIO.LOW)
    if log:
        print("low")

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(DIR, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(STEP, GPIO.OUT, initial=GPIO.LOW)

while True:
    for x in range(stepsPerRevolution):
        high(STEP)
        delay_ms(100)
        low(STEP)
        delay_ms(100)
    print("cycle complete")
    time.sleep(1)
