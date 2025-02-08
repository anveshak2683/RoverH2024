#!/usr/bin/env python3
import rospy
import time
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
from std_msgs.msg import Float32  # ROS message type for publishing timing

def get_frame(pipeline):
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    align = rs.align(rs.stream.color)
    frames = align.process(frames)
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    if not depth_frame or not color_frame:
        return False, None, None, None

    return True, depth_image, color_image, depth_frame

def arrowdetectmorethan3(pipeline, model, timing_pub):
    while not rospy.is_shutdown():  # Loop until ROS is shut down
        start_time = time.time()  # Start timer

        # Get frames
        ret, depth_image, color_image, depth_frame = get_frame(pipeline)
        if not ret:
            continue

        img = color_image
        results = model.predict(img, conf=0.2, max_det=2)

        arrow_in_center = False
        depth = 0

        for r in results:
            boxes = r.boxes
            for box in boxes:
                b = box.xyxy[0]  # Bounding box coordinates
                left, top, right, bottom = map(int, b)
                arrow_center = (left + right) / 2

                try:
                    depth = depth_frame.get_distance((left + right) // 2, (top + bottom) // 2)
                except:
                    pass

                img_center_x = img.shape[1] // 2
                if ((arrow_center - img_center_x) > -150 and (arrow_center - img_center_x) < 125) or depth < 3:
                    arrow_in_center = True

        end_time = time.time()  # End timer
        processing_time = end_time - start_time  # Calculate elapsed time
        rospy.loginfo(f"Time taken for one frame: {processing_time:.4f} seconds")

        # Optionally, publish the processing time as a ROS topic
        timing_pub.publish(processing_time)

        # Display the YOLO detection result
        cv2.imshow('YOLO V8 Detection', img)
        if cv2.waitKey(1) & 0xFF == ord(' '):
            break

    cv2.destroyAllWindows()

def main():
    # Initialize the ROS node
    rospy.init_node('arrow_detection_node')

    # Create a ROS publisher to publish timing info (optional)
    timing_pub = rospy.Publisher('/processing_time', Float32, queue_size=10)

    # Load YOLO model
    model = YOLO('/home/nvidia/caesar2020/auto/best.pt')

    # Initialize RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)

    try:
        # Call the arrow detection function
        arrowdetectmorethan3(pipeline, model, timing_pub)
    except rospy.ROSInterruptException:
        pass
    finally:
        pipeline.stop()

if __name__ == '__main__':
    main()

