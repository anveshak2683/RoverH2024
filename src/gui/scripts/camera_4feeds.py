import cv2
import numpy as np

# Initialize the video capture objects for four cameras
cam1 = cv2.VideoCapture(0)  # First camera
cam2 = cv2.VideoCapture(2)  # Second camera
cam3 = cv2.VideoCapture(4)  # Third camera
cam4 = cv2.VideoCapture(6)  # Fourth camera

while True:
    # Capture frames from each camera
    ret1, frame1 = cam1.read()
    ret2, frame2 = cam2.read()
    ret3, frame3 = cam3.read()
    ret4, frame4 = cam4.read()

    # Check if the frames were captured successfully, else set a black frame
    if not ret1:
        print("cam1")
        frame1 = np.zeros((360, 640, 3), dtype=np.uint8)
    if not ret2:
        print("cam2")
        frame2 = np.zeros((360, 640, 3), dtype=np.uint8)
    if not ret3:
        print("cam3")
        frame3 = np.zeros((360, 640, 3), dtype=np.uint8)
    if not ret4:
        print("cam4")
        frame4 = np.zeros((360, 640, 3), dtype=np.uint8)
    
    # Resize the frames to half size for displaying in a 2x2 grid
    frame1 = cv2.resize(frame1, (640, 360))
    frame2 = cv2.resize(frame2, (640, 360))
    frame3 = cv2.resize(frame3, (640, 360))
    frame4 = cv2.resize(frame4, (640, 360))
    
    # Create the 2x2 grid of frames
    top_row = np.hstack((frame1, frame2))
    bottom_row = np.hstack((frame3, frame4))
    grid = np.vstack((top_row, bottom_row))
    
    # Display the grid
    cv2.imshow('4 Camera Streams', grid)

    # Exit if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release all cameras and close windows
cam1.release()
cam2.release()
cam3.release()
cam4.release()
cv2.destroyAllWindows()

