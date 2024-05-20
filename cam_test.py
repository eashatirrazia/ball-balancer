from picamera2 import Picamera2 as PiCamera
import cv2
import time

camera = PiCamera()

vid = cv2.VideoCapture(0, cv2.CAP_V4L2)

if not vid.isOpened():
	print("Camera not read")
	
else:
	while(True):
		ret, frame = vid.read()
		cv2.imshow('frame', frame)
		print("hello")
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	vid.release()
	cv2.destroyAllWindows()
