#!/usr/bin/env python3
import copy
import sys
import rospy
import os
# import rosbag
import math
import time
import numpy as np
from std_msgs.msg import Bool

import threading
import std_msgs.msg as std_msgs
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from cv_bridge import CvBridge
import numpy as np
#rom ultralytics.utils.plotting import Annotator
from collections import defaultdict


# import statistics
import numpy as np
import cv2 as cv
#import open3d as o3d
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
os.chdir('/home/nvidia/caesar2020/src/juniors_autonomous')
 
# termination criteria
class ZedDepth:


    def __init__(self):
        self.color_image = None
        self.bridge = CvBridge()
        self.templatel = cv.imread('arrow_template_left.jpg', cv.IMREAD_GRAYSCALE)
        self.templater = cv.imread('arrow_template_right.jpg',cv.IMREAD_GRAYSCALE)
        assert self.templatel is not None, "file could not be read, check with os.path.exists()"
        rospy.Subscriber("/zed2i/zed_node/rgb/image_rect_color", Image, self.color_callback)
        print("color")

    def color_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # print(self.color_image[0][0])
            self.image_avbl = True
        except Exception as e:
            rospy.logerr("Failed to convert image: %s", e)

    def main(self):
        check = True
        dirn=None
        ret=False
        max_val_l,min_val_l,min_loc_l,max_loc_l = 0.0,0.0,0.0,0.0
        i_max = 0.0
        vals = np.arange(0.02,0.3,0.02)
        z=None
        # framecheck, frame = self.video_feed.read()# Read a single frame
        # if(framecheck==False):
        #     print("meow")
        #     return ret,dirn,None,z
        if self.color_image is None:
            print("No image")
            return 0
        img = self.color_image.copy()
        if img is None:
            print("meaoooww")
            return ret, dirn, None, z
        self.hp_cam_img2 = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        #self.hp_cam_img2 = self.hp_cam_img2[80:280,220:420]
        for i in vals:
            try:
                template2 = cv.resize(self.templatel, None, fx=i, fy=i, interpolation=cv.INTER_AREA)
                wl, hl = template2.shape[::-1]

                method = cv.TM_CCOEFF_NORMED
            
                # Apply template Matching
                res = cv.matchTemplate(self.hp_cam_img2,template2,method)
            
                min_val, max_val, min_loc, max_loc = cv.minMaxLoc(res)
                if(float(max_val)>float(max_val_l)):
                    min_val_l, max_val_l, min_loc_l, max_loc_l = min_val, max_val, min_loc, max_loc
                    w_finall,h_finall = wl, hl
                    i_max = i
            except cv.error as e:
                print(f"OpenCV error at scale {i}: {e}")
                continue


        max_val_r,min_val_r,min_loc_r,max_loc_r = 0.0,0.0,0.0,0.0
        i_max = 0.0

        for i in vals:
            try:
                template3 = cv.resize(self.templater, None, fx=i, fy=i, interpolation=cv.INTER_AREA)
                wr, hr = template3.shape[::-1]

                method = cv.TM_CCOEFF_NORMED
            
                # Apply template Matching
                res = cv.matchTemplate(self.hp_cam_img2,template3,method)
            
                min_val, max_val, min_loc, max_loc = cv.minMaxLoc(res)
                if(float(max_val)>float(max_val_r)):
                    min_val_r, max_val_r, min_loc_r, max_loc_r = min_val, max_val, min_loc, max_loc
                    w_finalr,h_finalr = wr, hr
                    i_max = i
            except cv.error as e:
                print(f"OpenCV error at scale {i}: {e}")
                continue


    
        #print(i_max)
        if max_loc_l != 0 or max_loc_r != 0:
            if(max_val_l>max_val_r):
                dirn = "left"
                top_left = max_loc_l
                bottom_right = (top_left[0] + w_finall, top_left[1] + h_finall)
                if float(max_val_l)>=0.61:
                    cv.rectangle(self.hp_cam_img2,top_left, bottom_right, 120, 2)
                max_val_c = max_val_l
            else:
                dirn = "right"
                top_left = max_loc_r
                bottom_right = (top_left[0] + w_finalr, top_left[1] + h_finalr)
                if float(max_val_r)>=0.61:
                    cv.rectangle(self.hp_cam_img2,top_left, bottom_right, 120, 2)
                max_val_c = max_val_l
        else:
            check = False
        # cv.imshow('frame',img2)
        # cv.waitKey(0)

        ## Depth calculation
        if(check):
            img_height = abs(top_left[1]-bottom_right[1])
            img_width = abs(top_left[0]-bottom_right[0])
            point = (int((img_width) / 2), int((img_height) / 2))

            try:
                z = self.depth_image[point[0], point[1]]

            except:
                self.ret = False
                # self.search()
                z = 2.5
            # img_height = abs(top_left[1]-bottom_right[1])
        
            # known_height = 21

            # f = 1117.60839

            # z = (f*known_height)/(img_height)
            conf = (round(float(max_val_c)*100))

            conf_str = str(conf)+"%"

            if(conf>70):
                # cv.putText(self.hp_cam_img2, str(z), (100,100), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                # cv.putText(self.hp_cam_img2, conf_str, (500,100), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                ret=True
                    # cv.putText(self.hp_cam_img2, dirn, (800,100), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                print("Direction:",dirn)
            else:
                dirn=None
            print("Confidence:", conf)
            print(self.hp_cam_img2.shape)
            cv.imshow('frame',self.hp_cam_img2)
            cv.waitKey(1)

    def run(self):
    # Main loop
        rate = rospy.Rate(10) 
        while not rospy.is_shutdown():
            print("goin in")
            self.main()
            rate.sleep()

if __name__ == '__main__':
    try:

        rospy.init_node('zed_depth', anonymous=True)
        rate = rospy.Rate(10) 
        fusion_node = ZedDepth()
        fusion_node.run()
    except rospy.ROSInterruptException:
        pass
    # finally:
    #     cv.destroyAllWindows()
