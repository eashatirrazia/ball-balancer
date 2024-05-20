import numpy as np
import cv2
import os
from pupil_apriltags import Detector
from picamera2 import Picamera2

picam2 = Picamera2()
picam2.preview_configuration.main.size = (800,800)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()



#os.add_dll_directory(r"C:\Users\DELL\anaconda3\envs\tf\lib\site-packages\pupil_apriltags.libs")

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
    kf.statePost = np.random.randn(4, 1).astype(np.float32)  # Random initial state

    return kf

# Initialize Kalman Filter
kf = initialize_kalman()


at_detector = Detector()

at_detector = Detector(
   families="tag25h9",
   nthreads=1,
   quad_decimate=1.0,
   quad_sigma=0.0,
   refine_edges=1,
   decode_sharpening=0.25,
   debug=0
)

#load the video
#cap = cv2.VideoCapture('april_video4.mp4')
while True:
   #ret, frame = cap.read()
   #if not ret:
   #   break
   frame = picam2.capture_array();
   gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
   detections = at_detector.detect(gray)
   #print(detections)
   # Prediction step of the Kalman Filter
   predicted = kf.predict()
   #if not detections:
      #cv2.circle(frame, (int(predicted[0]), int(predicted[1])), 5, (0, 255, 0), -1)
   for detection in detections:
      print(detection)
      corners = detection.corners
      if corners is not None:
         # Update step of the Kalman Filter
         measurement = np.array([[(corners[0][0] + corners[2][0]) / 2], [(corners[0][1] + corners[2][1]) / 2]], np.float32)
         #cv2.circle(frame, (int(measurement[0]), int(measurement[1])), 5, (0, 0, 255), -1)
         kf.correct(measurement)
         #for i in range(len(corners)):
               #cv2.line(frame, tuple(corners[i - 1, :].astype(int)), tuple(corners[i, :].astype(int)), (0, 0, 255), 2)
         # Draw the predicted position
      else:
         print("No tag detected")
         #cv2.circle(frame, (int(predicted[0]), int(predicted[1])), 5, (0, 255, 0), -1)
         # Draw the measured position
   #cv2.imshow('frame', frame)
   #if cv2.waitKey(0) & 0xFF == ord('q'):
      #break

cv2.destroyAllWindows()

