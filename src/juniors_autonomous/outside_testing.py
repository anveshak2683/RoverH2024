#!/usr/bin/env python3
import sys
import rospy
import math
import time
import cv2
import numpy as np
import imutils
import pyrealsense2 as rs
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator
from collections import defaultdict

model = YOLO('home/caesar2020/auto/best.pt') #old model
# model = YOLO('/home/kavin/Downloads/best.pt')
#model = YOLO('/home/kavin/Downloads/arrowspt2/runs/detect/train8/weights/best.pt') #new model
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

def get_frame():
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        align=rs.align(rs.stream.color)
        frames=align.process(frames)
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        if not depth_frame or not color_frame:
            return False, None, None, None
        return True, depth_image, color_image, depth_frame


def arrowdetectmorethan3():
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            img = np.asanyarray(color_frame.get_data())

            results = model.predict(img, conf = 0.5, max_det = 2)
            depth = 0
            arrow_in_center = False
            arrow_center = None
            if results == None:
                arrow_detected = "Not detected"
            else:
                arrow_detected = "Detected"
            
            #arrow_in_center = False

            for r in results:
                annotator = Annotator(img)
                boxes = r.boxes
                for box in boxes:
                    b = box.xyxy[0]  # get box coordinates in (left, top, right, bottom) format
                    c = box.cls
                    annotator.box_label(b, model.names[int(c)])

                    # Get dep/home/kavin/caesar2020_nvidia/src/navigation/scripts/kavin_modifi_2.pyth data for the bounding box
                    left, top, right, bottom = map(int, b)
                    arrow_center = (left+right)/2
                    try:
                        depth = depth_frame.get_distance((left + right) // 2, (top + bottom) // 2)
                    except:
                        pass
                    #print("Depth for box" + str(b) + ":" + str(depth) +"meters")

                    # Check if the arrow is in the center along the x-axis
                    img_center_x = 320
                    # img_center_x = img.shape[1] // 2
                    # Calculate the center along x-axis
                    #if left <= img_center_x <= right:
                    #print("arrow coordinates relative to center ", arrow_center-img_center_x)
                    if ((arrow_center-img_center_x)>-150) and (arrow_center - img_center_x)<125 or depth<3 :
                    # if depth<25:
                        arrow_in_center = True
                    else:
                        arrow_in_center = False
                    print(arrow_center-img_center_x)
            # print("imshow")
            cv2.imshow('YOLO V8 Detection', img)   
            if cv2.waitKey(1) & 0xFF == ord(' '):
               break         
            
            print("arrow in center: ", arrow_in_center)  
            if arrow_in_center:
                #cv2.putText(img, "Arrow in centre", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                print("Depth from more than 3:", depth)
                return arrow_in_center, "Not Available", arrow_center, depth
            else:
                return False, "Not available", None, 2.5
            
            
            
            print("imshow")
            cv2.imshow('YOLO V8 Detection', img)            
            return arrow_detected, "Not Available", aSrrow_center, depth
        # Stop streaming
        pipeline.stop()
        cv2.destroyAllWindows()
        #return False, "Not available", None, 0.0

def main():
	ret, depth_frame, color_frame, depth = get_frame()
	ret,direction,pix,distance=arrowdetectmorethan3()
	while(True):        
		a,b,c,d=arrowdetectmorethan3()
		print(a,b,c,d)
