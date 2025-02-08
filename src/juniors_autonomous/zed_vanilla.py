#!/usr/bin/env python

import rospy
import statistics
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
model = YOLO("/home/sparsh/Downloads/best_cube.pt") 


class ZedDepth:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('zed_depth', anonymous=True)

        # Subscribers
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.depth_sub = rospy.Subscriber("/zed2i/zed_node/depth/depth_registered", Image, self.depth_callback)

        # CvBridge for image conversion
        self.bridge = CvBridge()
        self.depth = 0
        # Variables to store latest data
        self.latest_image = None
        self.depth_image = None
        self.latest_xmin = 0
        self.latest_ymin = 0
        self.latest_xmax = 0
        self.latest_ymax = 0
        self.annotated_image = None
        self.results= None
        self.first_few = 20
    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        try:
            print("entered try1")
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            #  for r in results:
            #     self.annotated_image = r.plot()

            #     boxes = r.boxes
            #     for box in boxes:
            #         b = box.xyxy[0]
            #         left, top, right, bottom = map(int, b)
            #         self.latest_xmin = left
            #         self.latest_xmax = right
            #         self.latest_ymin = top
            #         self.latest_ymax = bottom
            #    cv2.imshow('text', self.annotated_image)
            # cv2.waitKey(1)
            
        except Exception as e:
            rospy.logerr("Failed to convert image: %s", e)
            
    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logerr(f"Error in depth_callback: {e}")
     
    
    def get_box(self):
        self.results = model.predict(self.latest_image, conf=0.5, max_det=2)
        if self.results!=None:        	
            for r in self.results:
                    self.annotated_image = r.plot()

                    boxes = r.boxes
                    for box in boxes:
                        b = box.xyxy[0]
                        left, top, right, bottom = map(int, b)
                        self.latest_xmin = left
                        self.latest_xmax = right
                        self.latest_ymin = top
                        self.latest_ymax = bottom
                    cv2.imshow('text', self.annotated_image)
            cv2.waitKey(1)

    

    def process_data(self):
        print("Entered process data")
        if self.latest_image is None or self.depth_image is None or self.results is None:
            print("returning nothing")
        
        print("x:", (self.latest_xmin + self.latest_xmax) // 2)
        print("y:", (self.latest_ymin + self.latest_ymax) // 2 )
        self.depth = self.depth_image[(self.latest_ymin + self.latest_ymax) // 2, (self.latest_xmin + self.latest_xmax) // 2]
        print(self.depth)
        """if self.latest_image is None or self.latest_points_3d is None or self.results is None :
            print("returning nothing")
            return  # Wait until both data sources are available
        # Transform LiDAR points to camera frame
        points_camera = (R @ self.latest_points_3d.T + T).T

        # Keep only points in front of the camera
        points_camera = points_camera[points_camera[:, 2] > 0]

        # Project points to 2D
        points_2d = (camera_matrix @ points_camera.T).T
        points_2d[:, 0] /= points_2d[:, 2]
        points_2d[:, 1] /= points_2d[:, 2]
        # Overlay points on the image
        image = self.latest_image.copy()
        print("I crossed check")
        #dist = 1000000000
        dist_list = []
        dist = 0.0
        
        n_pts = 0
        for pt in points_2d:
            x, y = int(pt[0]), int(pt[1])
            if self.latest_xmin <= x < self.latest_xmax and self.latest_ymin <= y < self.latest_ymax:
                pt[2] = round(pt[2], 1)
                dist_list.append(pt[2])
                n_pts += 1
                cv2.circle(self.annotated_image, (x, y), 2, (0, 255, 0), -1)
                     
        print("Number of points:", n_pts)
        
        # Calculate and print the mode of dist_list
        if dist_list:
            try:
                mode_dist = statistics.mode(dist_list)
                print("Mode of distances:", mode_dist)
            except statistics.StatisticsError:
                print("No unique mode found")
        else:
            print("No points found to calculate mode")"""
        # for pt in points_2d:
        #     x, y = int(pt[0]), int(pt[1])
        #     if self.latest_xmin <= x < self.latest_xmax and self.latest_ymin <= y < self.latest_ymax:
        #         # if dist > pt[2]:
        #         #       dist = (pt[2])
        #         pt[2]=round(pt[2],1)
        #         #dist+=pt[2]
                 
        #         dist_list.append(pt[2])
        #         n_pts+=1
        #         cv2.circle(self.annotated_image, (x, y), 2, (0, 255, 0), -1)
        #     #     cv2.imshow("Pointcloud", self.annotated_image)
        #     # cv2.waitKey(1)
        
        # print("no.of points:", n_pts)
        # # if n_pts != 0:
        # #     print(dist)
        # # if n_pts >= self.first_few:
        # #     dist = 0
        # #     dist_list = sorted(dist_list)
            
        # #     for i in range(self.first_few):
        # #         dist+=dist_list[i]
                
        # #     print(dist/self.first_few)
        # if n_pts!=0:
        #     print(dist/n_pts)
        
        # Display the fused image
        # text = f"Arrow"
        # # cv2.rectangle(image, (self.latest_xmin, self.latest_ymin, self.latest_xmax, self.latest_ymax))
        # cv2.rectangle(image, (self.latest_xmin, self.latest_ymin), (self.latest_xmax, self.latest_ymax), (0, 255, 0), 2)

        # # cv2.putText(image, text, (self.latest_xmin,self.latest_ymin))
        # cv2.imshow("Lidar-Camera Fusion", image)
        # cv2.waitKey(1)

    def run(self):
        # Main loop
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            print("goin in")
            self.get_box()
            self.process_data()
            rate.sleep()

if __name__ == '__main__':
    try:
        fusion_node = ZedDepth()
        fusion_node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
