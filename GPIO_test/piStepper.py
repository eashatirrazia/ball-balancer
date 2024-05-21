import RPi.GPIO as GPIO

from RpiMotorLib import RpiMotorLib

Gpio_pins=(-1,-1,-1)
direction=8
step=10

mymotor=RpiMotorLib.A4988Nema(direction,step,Gpio_pins,"A4988")

mymotor.motor_go(False,"1/16",100,50e-6,False,.05)
