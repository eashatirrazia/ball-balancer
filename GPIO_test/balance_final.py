import time
import math
from machine import Machine
from machine import A, B, C
from stepper import Stepper
import numpy as np
import cv2
from pupil_apriltags import Detector
from picamera2 import Picamera2

picam2 = Picamera2()
picam2.preview_configuration.main.size = (800,800)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

PADDING = 10
# Ball detection Params
dp = 1.2
dist = 150
param1 = 100
param2 = 35
minRadius = 10
maxRadius = 50
kernel_size = 7

REF_TAG_ID = 3

at_detector = Detector(
    families="tag25h9",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)

def initialize_kalman():
    ''' Initialize and return a Kalman Filter for tracking a ball in 2D space. '''
    kf = cv2.KalmanFilter(4, 2)  # 4 state parameters, 2 measurement parameters
    kf.transitionMatrix = np.array([[1, 0, 1, 0],
                                    [0, 1, 0, 1],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]], np.float32)
    kf.measurementMatrix = np.array([[1, 0, 0, 0],
                                     [0, 1, 0, 0]], np.float32)
    kf.processNoiseCov = np.eye(4, dtype=np.float32) * 0.2
    kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1
    kf.errorCovPost = np.eye(4, dtype=np.float32)
    kf.statePost = np.zeros((4, 1)).astype(
        np.float32)  # Random initial state

    return kf

tag_ids = [0, 1, 3, 4]
tag_centers = {tag_id: None for tag_id in tag_ids}
previous_centers = {tag_id: None for tag_id in tag_ids}
tag_corners = {tag_id: None for tag_id in tag_ids}
previous_corners = {tag_id: None for tag_id in tag_ids}
tag_id2corner = {0: 1, 1: 3, 3: 0, 4: 1}

kf = initialize_kalman()

def getCentroid(frame, kf, dp=1.2, dist=100, param1=100, param2=50, minRadius=35, maxRadius=60, kernel_size=9):
    '''
    Function to detect the centroid of a ball in a frame using Hough Circle Transform
    frame: The frame in which the ball is to be detected
    dp: Inverse ratio of the accumulator resolution to the image resolution
    dist: Minimum distance between the centers of the detected circles
    param1: Sensitivity of the edge detection in the Canny edge detector
    param2: Number of points that must be in a circle to be considered a circle
    minRadius: Minimum circle radius
    maxRadius: Maximum circle radius
    return: The frame with the detected circle and the centroid coordinates
    '''
    assert kernel_size % 2 == 1, "Kernel size must be an odd number"

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Apply Gaussian blur
    gray_blurred = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)
    # Apply Hough Circle Transform
    circles = cv2.HoughCircles(gray_blurred, cv2.HOUGH_GRADIENT, dp, dist, param1=param1, param2=param2, minRadius=minRadius, maxRadius=maxRadius)

    # Prediction step of the Kalman Filter
    predicted = kf.predict()

    # Measurement update step if circles are detected
    detected = False
    if circles is not None:
        circles = np.uint16(np.around(circles))
        measurement = np.array([[circles[0, 0][0]], [circles[0, 0][1]]], np.float32)
        kf.correct(measurement)
        centroid = (circles[0, 0][0], circles[0, 0][1])
        detected = True
    else:
        # If no circles detected, use the predicted state
        centroid = (int(predicted[0][0]), int(predicted[1][0]))

    # Draw the predicted state as a red circle
    # cv2.circle(frame, (int(predicted[0][0]), int(predicted[1][0])), 5, (255, 0, 0), 2)

    return frame, centroid, detected

def extract_coordinates():
    frame = picam2.capture_array()
    org_frame = frame.copy()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    detections = at_detector.detect(gray)
    detected_tag_ids = [detection.tag_id for detection in detections]

    for tag_id, center in previous_centers.items():
        if tag_id in detected_tag_ids:
            detection = next(d for d in detections if d.tag_id == tag_id)
            center = detection.center
            corners = detection.corners
            tag_center = center
            previous_centers[tag_id] = tag_center  # Update previous center
            required_corner = corners[tag_id2corner[tag_id]]
            previous_corners[tag_id] = required_corner
            
        else:
            tag_center = previous_centers[tag_id]
            required_corner = previous_corners[tag_id]

        tag_centers[tag_id] = tag_center
        tag_corners[tag_id] = required_corner
        

    # set box to corners of the whole frame
    box = np.array([[0, 0], [0, frame.shape[0]], [frame.shape[1], 0], [frame.shape[1], frame.shape[0]]], np.int32)
    box_center = np.mean(box, axis=0).astype(np.int32)
    
    
    # draw a box using tag_centers in this order (0, 3, 1, 4)
    # check if any of the tag_centers is None
    if all(corner is not None for corner in tag_corners.values()):
        box = np.array([tag_corners[tag_id]
                    for tag_id in [0, 3, 1, 4]], np.int32)
        # find the center of the box
        box_center = np.mean(box, axis=0).astype(np.int32)
    
    
    # extract the frame bounded by the box
    x, y, w, h = cv2.boundingRect(box)
    w_padded = w + PADDING
    h_padded = h + PADDING
    roi = org_frame[y:y+h_padded, x:x+w_padded]    
    frame_ball, centroid, detected = getCentroid(roi, kf, dp, dist, param1, param2, minRadius, maxRadius, kernel_size)
    
    # ref_origin = tag_corners[REF_TAG_ID]
    ref_origin = box_center
    relative_centroid = (centroid[0] - ref_origin[0]+ x, centroid[1] - ref_origin[1]+y)
    
    print("Relative Centroid: ", relative_centroid)
    
    return (relative_centroid[0], relative_centroid[1])


# Initialize machine object
machine = Machine(2, 3.125, 1.75, 3.669291339)  # (d, e, f, g)

stepper_pins = {
    'A': (8, 10),
    'B': (3, 5),
    'C': (11, 13)
}

# Initialize the steppers
stepperA = Stepper(stepper_pins['A'][1], stepper_pins['A'][0])
stepperB = Stepper(stepper_pins['B'][1], stepper_pins['B'][0])
stepperC = Stepper(stepper_pins['C'][1], stepper_pins['C'][0])

steppers = [stepperA, stepperB, stepperC]

steppers[0].set_speed(1/200)
steppers[1].set_speed(1/200)
steppers[2].set_speed(1/200)

# Stepper motor variables
pos = [0, 0, 0]  # Target positions for each stepper motor
ENA = 0  # Enable pin for the drivers
angOrig = 206.662752199  # Original angle that each leg starts at
speed = [0, 0, 0]
speedPrev = [0, 0, 0]
ks = 20  # Speed amplifying constant

# Touch screen variables
Xoffset = 0  # X offset for the center position of the touchpad
Yoffset = 0  # Y offset for the center position of the touchpad

# PID variables
kp = 4E-4
ki = 2E-6
kd = 7E-3  # PID constants
error = [0, 0]
errorPrev = [0, 0]
integr = [0, 0]
deriv = [0, 0]
out = [0, 0]  # PID terms for X and Y directions
timeI = 0  # Variables to capture initial times

# Other Variables
angToStep = 3200 / 360  # Angle to step conversion factor (steps per degree) for 16 microsteps or 3200 steps/rev
detected = False  # This value is True when the ball is detected and False when the ball is not detected

def setup():
    print("Initializing...")
    # Add setup code here
    time.sleep(1)  # Small delay to allow the user to reset the platform
    moveTo(4.25, 0, 0)  # Moves the platform to the home position

def loop():
    PID(0, 0)  # (X setpoint, Y setpoint) -- must be looped

# Moves/positions the platform with the given parameters
def moveTo(hz, nx, ny):
    global detected

    if detected:
        # Calculate stepper motor position
        for i in range(3):
            pos[i] = round((angOrig - machine.theta(i, hz, nx, ny)) * angToStep)
        # Set calculated speed
        set_max_speed(speed[A], speed[B], speed[C])
        # Set target positions
        move_to(pos[A], pos[B], pos[C])
    else:
        for i in range(3):
            pos[i] = round((angOrig - machine.theta(i, hz, 0, 0)) * angToStep)
        # Set max speed
        set_max_speed(800, 800, 800)
        # Move the stepper motors
        move_to(pos[A], pos[B], pos[C])

# Takes in an X and Y setpoint/position and moves the ball to that position
def PID(setpointX, setpointY):
    global detected, timeI

    p = extract_coordinates()  # Measure X and Y positions

    if p[0] != 0:
        detected = True
        for i in range(2):
            errorPrev[i] = error[i]
            error[i] = (i == 0) * (Xoffset - p[0] - setpointX) + (i == 1) * (Yoffset - p[1] - setpointY)
            integr[i] += error[i] + errorPrev[i]
            deriv[i] = error[i] - errorPrev[i]
            deriv[i] = 0 if math.isnan(deriv[i]) or math.isinf(deriv[i]) else deriv[i]
            out[i] = kp * error[i] + ki * integr[i] + kd * deriv[i]
            out[i] = max(-0.25, min(0.25, out[i]))
        
        for i in range(3):
            speedPrev[i] = speed[i]
            speed[i] = (i == A) * get_current_position(stepperA) + (i == B) * get_current_position(stepperB) + (i == C) * get_current_position(stepperC)
            speed[i] = abs(speed[i] - pos[i]) * ks
            speed[i] = max(speedPrev[i] - 200, min(speedPrev[i] + 200, speed[i]))
            speed[i] = max(0, min(1000, speed[i]))

        print(f"X OUT = {out[0]}   Y OUT = {out[1]}   Speed A: {speed[A]}")
    else:
        time.sleep(0.01)  # 10 millis delay before another reading
        p = extract_coordinates()
        if p[0] == 0:
            detected = False

    timeI = time.time()
    while (time.time() - timeI) < 0.02:
        moveTo(4.25, -out[0], -out[1])

def set_max_speed(a_speed, b_speed, c_speed):
    stepperA.set_speed(1/a_speed)
    stepperB.set_speed(1/b_speed)
    stepperC.set_speed(1/c_speed)

def move_to(a_pos, b_pos, c_pos):
    stepperA.move_to(a_pos)
    stepperB.move_to(b_pos)
    stepperC.move_to(c_pos)

def get_current_position(stepper):
    return stepper.position

# Example usage
setup()
while True:
    loop()
