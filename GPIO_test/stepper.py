import RPi.GPIO as GPIO
import time

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


def set_pin(pin, value):
	GPIO.output(pin, GPIO.LOW if value == 0 else GPIO.HIGH)

class Stepper:
	def __init__(self,step_pin, dir_pin):
		self.step_pin = step_pin
		self.dir_pin = dir_pin
		GPIO.setup(dir_pin, GPIO.OUT, initial=GPIO.LOW)
		GPIO.setup(step_pin, GPIO.OUT, initial=GPIO.LOW)
		self.position = 0
		self.delay = None 
		
	def set_speed(self, speed):
		self.delay = 1/abs(speed)

	def set_delay(self, delay):
		self.delay = delay

	def set_direction(self,direction):
		set_pin(self.dir_pin, direction)

	def step(self, delay=None):
		delay = delay if delay else self.delay
		high(self.step_pin)
		delay_ms(delay)
		low(self.step_pin)
		delay_ms(delay)

	def move_to(self,position):
		assert(self.delay)
		self.set_direction(1 if position > self.position else 0)
		while self.position != position:
				self.step()
				self.position += 1 if position > self.position else -1

if __name__ == "__main__":
	GPIO.setmode(GPIO.BOARD)
	GPIO.setwarnings(False)
	DIR = 8
	STEP = 10
	stepsPerRevolution = 3200

	stepper = Stepper(STEP, DIR)
	stepper.set_speed(1/100)

	stepper.move_to(800)
	time.sleep(1)

	stepper.move_to(0)
	time.sleep(1)
	
	DIR = 3
	STEP = 5
	stepsPerRevolution = 3200

	stepper = Stepper(STEP, DIR)
	stepper.set_speed(1/100)

	stepper.move_to(800)
	time.sleep(1)

	stepper.move_to(0)
	time.sleep(1)
	
	DIR = 11
	STEP = 13
	stepsPerRevolution = 3200

	stepper = Stepper(STEP, DIR)
	stepper.set_speed(1/100)

	stepper.move_to(800)
	time.sleep(1)

	stepper.move_to(0)
	time.sleep(1)

