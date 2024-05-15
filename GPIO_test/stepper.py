import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
		
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
		high(STEP)
		delay_ms(delay)
		low(STEP)
		delay_ms(delay)

	def move_to(self,position):
		assert(self.delay)
		p1=(position-self.position)*0.1 
		p2=(position-self.position)*0.9 
		init_delay = 200
		slope1 = (self.delay-init_delay)/p1
		print("Slope1:", slope1)
		slope2 = (self.delay-init_delay)/(p2-position+self.position)
		print("Slope2:", slope2)
		p1 += self.position
		p2 += self.position
		self.set_direction(1 if position > self.position else 0)
		direction = 1 if position > self.position else -1
		print("loop1")
		while self.position != p1:
			self.step(init_delay)
			init_delay = init_delay + direction*slope1
			self.position += 1 if position > self.position else -1
			print(self.position, init_delay)
		
		print("loop2")
		while self.position != p2:
			self.step()
			self.position += 1 if position > self.position else -1
			print(self.position, self.delay)
		
		print("loop3")	
		init_delay = self.delay
		while self.position != position:
			self.step(init_delay)
			init_delay = init_delay + direction*slope2
			self.position += 1 if position > self.position else -1
			print(self.position, init_delay)
		
		print(f"Final Delay: {init_delay}")
		
DIR = 8
STEP = 10
stepsPerRevolution = 3200

stepper = Stepper(STEP, DIR)
stepper.set_speed(1/100)

stepper.move_to(1000)
time.sleep(1)

#stepper.move_to(3200)
#time.sleep(1)

#stepper.move_to(0)
#time.sleep(1)

