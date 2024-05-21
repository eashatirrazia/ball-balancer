import time
import math
#from RpiMotorLib import RpiMotorLib
import RPi.GPIO as GPIO
from stepper import Stepper

# Constants
A = 0
B = 1
C = 2

# Machine class (equivalent to InverseKinematics.h in Arduino)


class Machine:
    def __init__(self, d, e, f, g):
        self.d = d
        self.e = e
        self.f = f
        self.g = g

    def theta(self, leg, hz, nx, ny):
        nmag = math.sqrt(nx*2 + ny*2 + 1)
        nx /= nmag
        ny /= nmag
        nz = 1 / nmag

        if leg == A:
            y = self.d + (self.e / 2) * (1 - (nx*2 + 3 * nz*2 + 3 * nz) / (
                nz + 1 - nx*2 + (nx4 - 3 * nx2 * ny2) / ((nz + 1) * (nz + 1 - nx*2))))
            z = hz + self.e * ny
            mag = math.sqrt(y*2 + z*2)
            angle = math.acos(y / mag) + math.acos((mag**2 +
                                                    self.f*2 - self.g*2) / (2 * mag * self.f))

        elif leg == B:
            x = (math.sqrt(3) / 2) * (self.e *
                                      (1 - (nx**2 + math.sqrt(3) * nx * ny) / (nz + 1)) - self.d)
            y = x / math.sqrt(3)
            z = hz - (self.e / 2) * (math.sqrt(3) * nx + ny)
            mag = math.sqrt(x*2 + y2 + z*2)
            angle = math.acos((math.sqrt(3) * x + y) / (-2 * mag)) + \
                math.acos((mag*2 + self.f2 - self.g*2) /
                          (2 * mag * self.f))

        elif leg == C:
            x = (math.sqrt(3) / 2) * (self.d - self.e *
                                      (1 - (nx**2 - math.sqrt(3) * nx * ny) / (nz + 1)))
            y = -x / math.sqrt(3)
            z = hz + (self.e / 2) * (math.sqrt(3) * nx - ny)
            mag = math.sqrt(x*2 + y2 + z*2)
            angle = math.acos((math.sqrt(3) * x - y) / (2 * mag)) + \
                math.acos((mag*2 + self.f2 - self.g*2) /
                          (2 * mag * self.f))

        return math.degrees(angle)


# Define pins for stepper motors
stepper_pins = {
    'A': (8, 10),
    'B': (12, 13),
    'C': (21, 22)
}

# Initialize the steppers
stepperA = Stepper(stepper_pins['A'][0], stepper_pins['A'][1])
stepperB = Stepper(stepper_pins['B'][0], stepper_pins['B'][1])
stepperC = Stepper(stepper_pins['C'][0], stepper_pins['C'][1])

steppers = [stepperA, stepperB, stepperC]

steppers[0].set_speed(1/20)
steppers[1].set_speed(1/20)
steppers[2].set_speed(1/20)
# Initialize variables
angOrig = 206.662752199
speed = [0, 0, 0]
ks = 20
Xoffset = 500
Yoffset = 500
kp = 4E-4
ki = 2E-6
kd = 7E-3
error = [0, 0]
integr = [0, 0]
deriv = [0, 0]
out = [0, 0]
angToStep = 3200 / 360
detected = 0
pos = [0, 0, 0]
# Function to move platform
x = 0
y = 0


def moveTo(hz, nx, ny):
    global detected
    global pos

    if detected:
        for i in range(3):
            pos[i] = round(
                (angOrig - machine.theta(i, hz, nx, ny)) * angToStep)

        for i, stepper in enumerate(steppers):
            stepper.move_to(abs(pos[i]))

    else:
        for i in range(3):
            pos[i] = round((angOrig - machine.theta(i, hz, 0, 0)) * angToStep)

        for i, stepper in enumerate(steppers):
            stepper.move_to(abs(pos[i]))

# PID control function


def PID(setpointX, setpointY):
    global detected, error, integr, deriv, out, speed, pos

    if x != 0:
        detected = 1
        for i in range(2):
            errorPrev = error[i]
            error[i] = (i == 0) * (Xoffset - x - setpointX) + \
                (i == 1) * (Yoffset - y - setpointY)
            integr[i] += error[i] + errorPrev
            deriv[i] = error[i] - errorPrev
            if math.isnan(deriv[i]) or math.isinf(deriv[i]):
                deriv[i] = 0
            out[i] = kp * error[i] + ki * integr[i] + kd * deriv[i]
            out[i] = max(min(out[i], 0.25), -0.25)

        for i in range(3):
            speedPrev = speed[i]
            speed[i] = steppers[i].position
            speed[i] = abs(speed[i] - pos[i]) * ks
            speed[i] = max(min(speed[i], speedPrev + 200), speedPrev - 200)
            speed[i] = max(min(speed[i], 1000), 0)

        print(
            f"X OUT = {out[0]:.4f}, Y OUT = {out[1]:.4f}, Speed A: {speed[A]}")

    else:
        time.sleep(0.01)

        if x == 0:
            detected = 0

    timeI = time.time()
    while (time.time() - timeI) < 0.02:
        moveTo(4.25, -out[0], -out[1])


machine = Machine(2, 3.125, 1.75, 3.669291339)

GPIO.setmode(GPIO.BOARD)

#ENA = 0
#GPIO.setup(ENA, GPIO.OUT)
#GPIO.output(ENA, GPIO.HIGH)
#time.sleep(1)
#GPIO.output(ENA, GPIO.LOW)

moveTo(4.25, 0, 0)
#steppers[2].set_speed(1/20)
#steppers[2].move_to(6400*2)
while True:
    PID(0, 0)
