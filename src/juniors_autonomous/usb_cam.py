#!/usr/bin/env python

import rospy
import torch
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import ultralytics
from ultralytics import YOLO
model = YOLO("/home/nvidia/caesar2020/src/juniors_autonomous/best.pt") 

def image_callback(msg):
    # Initialize CvBridge
    bridge = CvBridge()

    # Convert the ROS image message to an OpenCV image (BGR format)
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        rospy.logerr("Error converting image: %s", str(e))
        return

    # Run inference using YOLOv8 model
    results = model.predict(cv_image, conf=0.5, max_det=2)

    # Render the results on the image (bounding boxes, labels, etc.)
    for r in results:
        annotated_image = r.plot()

    # If you want more control over what is displayed, you can do this:
        cv2.imshow('text', annotated_image)

    cv2.waitKey(1)

def main():
    # Initialize the ROS node
    rospy.init_node('usb_cam_arrow_detection', anonymous=True)

    # Subscribe to the /usb_cam/image_raw topic
    rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    main()
