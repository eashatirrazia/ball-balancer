import time
import math

# Constants
A = 0
B = 1
C = 2

# Machine class
class Machine:
    def __init__(self, d, e, f, g):
        self.d = d  # distance from the center of the base to any of its corners
        self.e = e  # distance from the center of the platform to any of its corners
        self.f = f  # length of link #1
        self.g = g  # length of link #2

    def theta(self, leg, hz, nx, ny):
        # create unit normal vector
        nmag = math.sqrt(nx ** 2 + ny ** 2 + 1)  # magnitude of the normal vector
        nx /= nmag
        ny /= nmag
        nz = 1 / nmag

        # calculates angle A, B, or C
        if leg == A:  # Leg A
            y = self.d + (self.e / 2) * (1 - ((nx ** 2 + 3 * nz ** 2 + 3 * nz) / 
                 (nz + 1 - nx ** 2 + ((nx ** 4 - 3 * nx ** 2 * ny ** 2) / ((nz + 1) * (nz + 1 - nx ** 2))))))
            z = hz + self.e * ny
            mag = math.sqrt(y ** 2 + z ** 2)
            angle = math.acos(y / mag) + math.acos((mag ** 2 + self.f ** 2 - self.g ** 2) / (2 * mag * self.f))

        elif leg == B:  # Leg B
            x = (math.sqrt(3) / 2) * (self.e * (1 - ((nx ** 2 + math.sqrt(3) * nx * ny) / (nz + 1))) - self.d)
            y = x / math.sqrt(3)
            z = hz - (self.e / 2) * (math.sqrt(3) * nx + ny)
            mag = math.sqrt(x ** 2 + y ** 2 + z ** 2)
            angle = math.acos((math.sqrt(3) * x + y) / (-2 * mag)) + math.acos((mag ** 2 + self.f ** 2 - self.g ** 2) / (2 * mag * self.f))

        elif leg == C:  # Leg C
            x = (math.sqrt(3) / 2) * (self.d - self.e * (1 - ((nx ** 2 - math.sqrt(3) * nx * ny) / (nz + 1))))
            y = -x / math.sqrt(3)
            z = hz + (self.e / 2) * (math.sqrt(3) * nx - ny)
            mag = math.sqrt(x ** 2 + y ** 2 + z ** 2)
            angle = math.acos((math.sqrt(3) * x - y) / (2 * mag)) + math.acos((mag ** 2 + self.f ** 2 - self.g ** 2) / (2 * mag * self.f))

        else:
            raise ValueError("Invalid leg identifier")

        return math.degrees(angle)  # converts angle to degrees and returns the value