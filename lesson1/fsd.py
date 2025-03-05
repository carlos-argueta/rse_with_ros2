import cv2
import numpy as np
import time

def play_video(video_path):
    """Plays a video and allows frame transformations with key presses.

    Keybindings:
        'r': Convert to RGB
        'b': Convert to BGR
        'g': Convert to Grayscale
        'f': Flip vertically
        'q': Quit
    """

    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        print(f"Error: Could not open video at {video_path}")
        return

    current_frame = None  # Store the current frame
    transformation = None # Store current transformation

    while True:
        ret, frame = cap.read()
        if not ret:
            break  # End of video

        current_frame = frame.copy() # important to copy the frame, otherwise the transformations will accumulate

        if transformation == "rgb":
            current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
        elif transformation == "gray":
            current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
        elif transformation == "flip":
            current_frame = cv2.flip(current_frame, 1)  # 0 for vertical flip

        cv2.imshow('Video Player', current_frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            transformation = "rgb"
        elif key == ord('b'):
            transformation = "bgr"
        elif key == ord('g'):
            transformation = "gray"
        elif key == ord('f'):
            transformation = "flip"

        time.sleep(0.05)  # Slow down the video

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    video_file = "dash1.mp4"  # Replace with your video file path
    play_video(video_file)