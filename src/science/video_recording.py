import cv2
import time

def record_video(output_filename, duration=10, camera_index=2):
    # Open the video capture device
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print("Error: Could not open the camera.")
        return

    # Get the default frame width and height
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv2.CAP_PROP_FPS)) or 30  # Default to 30 FPS if not available

    # Define the codec and create a VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Use XVID codec
    out = cv2.VideoWriter(output_filename, fourcc, fps, (frame_width, frame_height))

    print(f"Recording video for {duration} seconds...")
    start_time = time.time()
    a = 0
    while time.time() - start_time < duration:
        ret, frame = cap.read()
        if not ret:
            print("Error: Unable to read frame.")
            break

        # Write the frame to the output file
        out.write(frame)



        # Optionally show the frame while recording
        # cv2.imshow('Recording', frame)

        # Stop recording if 'q' is pressed
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break

        if a == 0 and time.time() - start_time > duration / 2:
            cv2.imwrite("/home/nvidia/output.jpg", frame)
            a = 1
    # Release the resources
    cap.release()
    out.release()
    cv2.destroyAllWindows()
    print(f"Video saved to {output_filename}")

# Example usage
record_video('output.mp4', duration=10, camera_index=2)
