import numpy as np
import cv2
from pupil_apriltags import Detector

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
        cv2.circle(frame, centroid, 5, (0, 255, 0), 2)
        cv2.circle(frame, centroid, circles[0, 0][2], (0, 255, 0), 3)
        detected = True
    else:
        # If no circles detected, use the predicted state
        centroid = (int(predicted[0][0]), int(predicted[1][0]))
        cv2.circle(frame, centroid, 5, (0, 0, 255), 2)

    # Draw the predicted state as a red circle
    # cv2.circle(frame, (int(predicted[0][0]), int(predicted[1][0])), 5, (255, 0, 0), 2)

    return frame, centroid, detected


at_detector = Detector(
    families="tag25h9",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)

tag_ids = [0, 1, 3, 4]
tag_centers = {tag_id: None for tag_id in tag_ids}
previous_centers = {tag_id: None for tag_id in tag_ids}
tag_corners = {tag_id: None for tag_id in tag_ids}
previous_corners = {tag_id: None for tag_id in tag_ids}
tag_id2corner = {0: 1, 1: 3, 3: 0, 4: 1}

kf = initialize_kalman()

cap = cv2.VideoCapture(1)

while True:
    ret, frame = cap.read()
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
            
            for i in range(len(corners)):
                cv2.line(frame, tuple(
                    corners[i - 1, :].astype(int)), tuple(corners[i, :].astype(int)), (0, 0, 255), 2)
        else:
            tag_center = previous_centers[tag_id]
            required_corner = previous_corners[tag_id]

        tag_centers[tag_id] = tag_center
        tag_corners[tag_id] = required_corner
        
        cv2.circle(frame, (int(required_corner[0]), int(
            required_corner[1])), 5, (0, 0, 255), -1)
        
        # Draw the tag center
        # cv2.circle(frame, (int(tag_center[0]), int(
        #     tag_center[1])), 5, (0, 0, 255), -1)
        cv2.putText(frame, str(tag_id), (int(tag_center[0]), int(
            tag_center[1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

    # set box to corners of the whole frame
    box = np.array([[0, 0], [0, frame.shape[0]], [frame.shape[1], 0], [frame.shape[1], frame.shape[0]]], np.int32)
    box_center = np.mean(box, axis=0).astype(np.int32)
    
    
    # draw a box using tag_centers in this order (0, 3, 1, 4)
    # check if any of the tag_centers is None
    if all(corner is not None for corner in tag_corners.values()):
        box = np.array([tag_corners[tag_id]
                       for tag_id in [0, 3, 1, 4]], np.int32)
        cv2.polylines(frame, [box], True, (0, 255, 0), 2)
        # find the center of the box
        box_center = np.mean(box, axis=0).astype(np.int32)
        
    cv2.putText(frame, f"({box_center[0]},{box_center[1]})", tuple(
        box_center), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)
    cv2.circle(frame, tuple(box_center), 2, (255, 0, 0), -1)
    
    # extract the frame bounded by the box
    x, y, w, h = cv2.boundingRect(box)
    w_padded = w + PADDING
    h_padded = h + PADDING
    roi = org_frame[y:y+h_padded, x:x+w_padded]    
    frame_ball, centroid, detected = getCentroid(roi,kf, dp, dist, param1, param2, minRadius, maxRadius, kernel_size)
    cv2.putText(roi, "(X, Y) = ({}, {})".format(centroid[0], centroid[1]), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    cv2.circle(roi, tuple(centroid), 5, (0, 255, 0), 2)
    
    ref_origin = tag_corners[REF_TAG_ID]
    relative_centroid = (centroid[0] - ref_origin[0]+ x, centroid[1] - ref_origin[1]+y)
    
    # draw vectors on the frame
    
    #1- from top left of frame to x,y
    cv2.arrowedLine(frame, (0,0), (x,y), (0, 255, 0), 2)
    cv2.putText(frame, "C", (x//2, y//2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
    
    #2- from top left of frame to ref_origin
    cv2.arrowedLine(frame, (0,0), (int(ref_origin[0]), int(ref_origin[1])), (0, 255, 0), 2)
    cv2.putText(frame, "D", (int(ref_origin[0]//2), int(ref_origin[1]//2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
    
    # from x,y to ref_origin
    cv2.arrowedLine(frame, (x,y), (int(ref_origin[0]), int(ref_origin[1])), (0, 0, 255), 2)
    cv2.putText(frame, "B", (int((x+ref_origin[0])//2), int((y+ref_origin[1])//2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
    
    # from ref_origin to centroid
    plot_centroid = (centroid[0] + x, centroid[1] + y)
    cv2.arrowedLine(frame, (int(ref_origin[0]), int(ref_origin[1])), tuple(plot_centroid), (255, 0, 255), 2)
    cv2.putText(frame, "x", (int((centroid[0] + ref_origin[0])//2), int((centroid[1] + ref_origin[1])//2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
    
    # from x,y to plot_centroid
    cv2.arrowedLine(frame, (x,y), tuple(plot_centroid), (255, 0, 255), 2)
    cv2.putText(frame, "A", (int((x+centroid[0])//2), int((y+centroid[1])//2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
    
    print("Relative Centroid: ", relative_centroid)
    
    cv2.imshow('roi', roi)
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()