#!/usr/bin/env python3
import sys
import rospy
import math
import time
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator

li=[]
# Function to capture and return frames
def get_frame(pipeline):
    frames = pipeline.wait_for_frames()
    align = rs.align(rs.stream.color)
    frames = align.process(frames)

    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    if not depth_frame or not color_frame:
        return False, None, None, None

    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    return True, depth_image, color_image, depth_frame


# Function to detect arrows and calculate distance
def arrowdetectmorethan3(pipeline, model):
    while True:
        ret, depth_image, color_image, depth_frame = get_frame(pipeline)
        if not ret:
            continue

        img = color_image
        results = model.predict(img, conf=0.35, max_det=2)

        depth = 0
        arrow_in_center = False	
        arrow_center = None

        for r in results:
            annotator = Annotator(img)
            boxes = r.boxes
            for box in boxes:
                b = box.xyxy[0]  # get box coordinates in (left, top, right, bottom) format
                c = box.cls
                annotator.box_label(b, model.names[int(c)])

                left, top, right, bottom = map(int, b)
                arrow_center = (left + right) / 2

                try:
                    depth = depth_frame.get_distance((left + right) // 2, (top + bottom) // 2)
                except Exception as e:
                    print(f"Error calculating depth: {e}")

                img_center_x = img.shape[1] // 2
                if -150 < (arrow_center - img_center_x) < 125 or depth < 3:
                    arrow_in_center = True
                else:
                    arrow_in_center = False

                print(f"Arrow center difference from image center: {arrow_center - img_center_x}")

        cv2.imshow('YOLO V8 Detection', img)

        # Press space to break the loop
        if cv2.waitKey(1) & 0xFF == ord(' '):
            break

        print("Arrow in center: ", arrow_in_center)
        if arrow_in_center:
            print("Depth from more than 3 meters:", depth)
            li.append(depth)
            return arrow_in_center, "Not Available", arrow_center, depth
        else:
            return False, "Not available", None, 2.5

    pipeline.stop()
    cv2.destroyAllWindows()


def main():
    # Load YOLO model and start RealSense pipeline
    model = YOLO('/home/nvidia/caesar2020/auto/best.pt')
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)
    

    while True:
        start_time = time.time()
        arrow_in_center, direction, pix, distance = arrowdetectmorethan3(pipeline, model)
        print(arrow_in_center, direction, pix, distance)
        end_time = time.time()  # End timer
        print(end_time-start_time)
        #print(max(li))


if __name__ == "__main__":
    main()

