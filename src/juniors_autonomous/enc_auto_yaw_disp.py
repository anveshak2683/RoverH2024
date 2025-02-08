#!/usr/bin/env python3
import copy
import sys
import rospy

# import rosbag
from navigation.msg import gps_data
import math
import time
import numpy as np
import imutils
from traversal.msg import WheelRpm
from traversal.srv import *
from std_msgs.msg import Bool
import pyrealsense2 as rs
import threading
import std_msgs.msg as std_msgs
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from ultralytics import YOLO
from cv_bridge import CvBridge
import numpy as np
import pyrealsense2 as rs
from ultralytics.utils.plotting import Annotator
from collections import defaultdict


import statistics
import numpy as np
import cv2 as cv
import open3d as o3d
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import csv


#============This is a hard coded version where there is prior knowledge of the arrow directions. The directions have to be fed in the form of strings into a list. To be used only if cv cups============

class ZedDepth:
    def __init__(self):
        # Initialize ROS node

        # Subscribers
        # CvBridge for image conversion
        self.bridge = CvBridge()
        self.depth = 0
        # Variables to store latest data
        self.color_image = None
        self.depth_image = None
        self.latest_xmin = 0
        self.latest_ymin = 0
        self.latest_xmax = 0
        self.latest_ymax = 0
        self.annotated_image = None
        self.results= None
        self.first_few = 20
        self.yaw_initialization_done = False
        #self.template_r = cv.imread("Template.png", 0)
        #self.template_l = cv.imread("Template_l.png", 0)
        #self.template_r = cv.resize(self.template_r, (60, 40), cv.INTER_AREA)
        #self.template_l = cv.resize(self.template_l, (60, 40), cv.INTER_AREA)
        #self.h, self.w = self.template_r.shape
        self.z_angle = self.x_angle = self.y_angle = 0
        self.turn = False
        self.circle_dist = 1.5
        self.dist_thresh = 0.3
        self.angle_thresh = 4
        self.kp = 20
        self.kp_rot = 1.5
        self.kp_straight_rot = 7.5
        self.distance = 10.0
        self.time_bool=False
        self.direction = "Not Available"
        self.current_latitude = 0.0
        self.current_longitude = 0.0
        self.ret = False
        self.initial_yaw = 0.0
        self.rotate_angle = 90
        self.angles_dict = defaultdict(list)
        self.searchcalled = False
        self.latlong = defaultdict(list)
        self.latlong[0] = "latitude"
        self.latlong[1] = "longitude"
        self.arrow_numbers = 1
        self.gpscalled = 0
        self.depth_image=None
        self.bridge = CvBridge()
        self.drift_correction_const = 32

        # bag
        #        self.num=i
        #        filename = "imu_data_"+str(self.num)+".bag"
        #        self.bag=rosbag.Bag(filename,'w')
        self.state = False
        self.rot = 0 #No steering mode 
        self.initial_drift_angle = 0

        # search alg by turning realsense
        self.enc_data = 0
        self.start_time = time.time()
        self.time_thresh = 20
        self.time_thresh_rot = 5
        self.pub = rospy.Publisher("stm_write", std_msgs.Int32MultiArray, queue_size=10)
        self.init = False
        self.start_angle = 50
        self.angle_thresh = 4
        # self.manjari = False
        self.count_arrow = 0
        #self.left_arrow_count = 0
        #self.right_arrow_count = 0
        self.arrows=["left"]*self.arrow_numbers
        self.image_avbl = False
        self.pls_call_rot_once=False
        
        self.base_index=1        #For base rotation
        self.base_rot_dir=1
        self.driv=None 

        try:
            rospy.Subscriber("/enc_auto", std_msgs.Float32MultiArray, self.drive_callback)
            print("1" )
            rospy.Subscriber("/zed2i/zed_node/imu/data", Imu, self.yaw_callback)
            print("2")
            rospy.Subscriber("enc_arm", std_msgs.Float32MultiArray, self.enc_callback)
            print("3")
        except KeyboardInterrupt:
            # quit
            sys.exit()
            
    def drive_callback(self,msg):
        self.driv=msg.data 
    def quaternion_to_euler(self,x, y, z, w):

# Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

# Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
            siny_cosp = 2 * (w * z + x * y)
            cosy_cosp = 1 - 2 * (y * y + z * z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
        #print("current yaw", yaw)
        return roll, pitch, yaw

    def yaw_callback(self,data):
        current_x = data.orientation.x
        current_y = data.orientation.y
        current_z = data.orientation.z
        current_w = data.orientation.w
        current_tuple=(current_x, current_y, current_z, current_w)
        current_pitch, current_roll, self.z_angle_in_rad = self.quaternion_to_euler(data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w)

        if self.yaw_initialization_done == False:
            self.initial_yaw = self.z_angle_in_rad*180/math.pi
            self.z_angle = self.z_angle_in_rad*180/math.pi - self.initial_yaw
            self.yaw_initialization_done = True
        else:
            self.z_angle = self.z_angle_in_rad*180/math.pi
        #print("self.z_angle in callback (in degrees)", self.z_angle)

        if self.z_angle < -179:
            self.z_angle = self.z_angle + 360
        elif self.z_angle > 179:
            self.z_angle = self.z_angle - 360
            

    def enc_callback(self, msg):
        #print(msg.data)
        #print("meao")
        self.enc_data = msg.data[1]

    def main(self):
        if(time.time()-self.start_time)>2:
            print()
            print(f"Yaw is {self.z_angle}")
            if self.driv is not None:
                print(self.driv)
                print(f"FRONT LEFT DRIVE: {self.driv[0]}                FRONT RIGHT DRIVE: {self.driv[1]}")
                print(f"BACK LEFT DRIVE: {self.driv[2]}                 BACK RIGHt DRIVE: {self.driv[3]}")
            self.start_time=time.time()
            print()
            print()
            print('--------------------------------------------------------------------------------------------------------------------------------------')
           
    def run(self):
        rate = rospy.Rate(10) 
        while not rospy.is_shutdown():
            self.main()
            rate.sleep()

if __name__ == '__main__':
    try:

        rospy.init_node('zed_depth', anonymous=True)
        rate = rospy.Rate(10) 
        wheelrpm_pub = rospy.Publisher("motion", WheelRpm, queue_size=10)
        gps_data_pub = rospy.Publisher("gps_bool", std_msgs.Int8, queue_size=10)
        rotin_pub = rospy.Publisher("rot", std_msgs.Int8, queue_size =10)
        fusion_node = ZedDepth()
        fusion_node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv.destroyAllWindows()


