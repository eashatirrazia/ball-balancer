# extract frames from a video
import cv2
import os

def extract_frames(video_path, output_folder):
    # check if the output folder exists
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # open the video
    cap = cv2.VideoCapture(video_path)
    frame_count = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        # save the frame
        frame_path = os.path.join(output_folder, f'frame_{frame_count}.jpg')
        cv2.imwrite(frame_path, frame)
        frame_count += 1
    cap.release()

if __name__ == '__main__':
    video_path = 'ball_video.mp4'
    output_folder = 'frames'
    extract_frames(video_path, output_folder)
    print('Frames extracted successfully')