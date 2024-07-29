import cv2
import sys

def capture_photo(webcam_index):
    # Initialize the webcam with the given index
    cap = cv2.VideoCapture(webcam_index)

    if not cap.isOpened():
        print(f"Error: Could not open webcam with index {webcam_index}.")
        return

    # Read a frame from the webcam
    ret, frame = cap.read()

    if not ret:
        print("Error: Could not read frame.")
        cap.release()
        return

    # Display the captured frame
    cv2.imshow('Captured Photo', frame)

    # Wait for a key press and then close the displayed window
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Release the webcam
    cap.release()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        #print("Usage: python script_name.py <webcam_index>")

        sys.exit(1)
    
    webcam_index = int(sys.argv[1])
    capture_photo(webcam_index)

