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

#================This is the standard code for rot in place===========================

model = YOLO("/home/nvidia/caesar2020/src/juniors_autonomous/best_cube.pt") 
cone_model = YOLO("/home/nvidia/caesar2020/src/juniors_autonomous/Cone.pt")

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
        self.templatel = cv.GaussianBlur(cv.imread('arrow_template_left.jpg', cv.IMREAD_GRAYSCALE),(5,5),3)
        self.templater = cv.GaussianBlur(cv.imread('arrow_template_right.jpg',cv.IMREAD_GRAYSCALE),(5,5),3)
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
        for i in range(5):
            print("hey! self.distance = 10", self.distance)
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
        self.image_avbl = False
        self.pls_call_rot_once=False
        
        self.base_index=1        #For base rotation
        self.base_rot_dir=1
        
        try:
            rospy.Subscriber("state", Bool, self.state_callback)
            print("1")
            rospy.Subscriber("/zed2i/zed_node/imu/data", Imu, self.yaw_callback)
            print("2")
            rospy.Subscriber("enc_arm", std_msgs.Float32MultiArray, self.enc_callback)
            print("3")
            rospy.Subscriber("/gps_coordinates", NavSatFix, self.gps_callback)
            print("4")
            rospy.Subscriber("/zed2i/zed_node/rgb/image_rect_color", Image, self.color_callback)
            print("color")
            rospy.Subscriber("/zed2i/zed_node/depth/depth_registered", Image, self.depth_callback)
            print("depth")
        except KeyboardInterrupt:
            # quit
            sys.exit()
            
        
    def state_callback(self, msg):
        self.state = False

    def color_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.color_image = self.color_image[30:330,170:490]
            self.image_avbl = True
        except Exception as e:
            rospy.logerr("Failed to convert image: %s", e)
            
    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.depth_image = self.depth_image[30:330,170:490]
        except Exception as e:
            rospy.logerr(f"Error in depth_callback: {e}")
     
    
    def get_box(self):
        self.results = model.predict(self.color_image, conf=0.5, max_det=2)
        
        if self.results!=None:        	
            for r in self.results:
                    print(f"Entered Get Box")
                    self.annotated_image = r.plot()
                    print("Inside For loop")
                    boxes = r.boxes
                    if len(boxes.conf) == 0:
                        print("No arrow detected")
                        self.ret = False
                    else:
                        self.ret = True
                    for box in boxes:
                        b = box.xyxy[0]
                        left, top, right, bottom = map(int, b)
                        self.latest_xmin = left
                        self.latest_xmax = right
                        self.latest_ymin = top
                        self.latest_ymax = bottom
                    cv.imshow('text', self.annotated_image)
                    print(f"Exited Forloop. self.ret = {self.ret}")
            cv.waitKey(1)
        else:
            self.ret = False

    
    def cone_model(self):
        print("Entered Cone Model Now")
        self.results = cone_model.predict(self.color_image, conf=0.7, max_det=2)
        if self.results!=None:        	
            for r in self.results:
                    print(f"Entered Get Box of cone model")
                    self.annotated_image = r.plot()
                    print("Inside For loop of cone model")
                    boxes = r.boxes
                    if len(boxes.conf) == 0:
                        print("No cone detected")
                        self.ret = False
                    else:
                        self.ret = True
                    for box in boxes:
                        b = box.xyxy[0]
                        left, top, right, bottom = map(int, b)
                        self.latest_xmin = left
                        self.latest_xmax = right
                        self.latest_ymin = top
                        self.latest_ymax = bottom
                    cv.imshow('text', self.annotated_image)
                    print(f"Exited Forloop of cone model. self.ret = {self.ret}")
            cv.waitKey(1)
        else:
            self.ret = False

        print("Entered process data")
        if self.color_image is None or self.depth_image is None or self.results is None:
            print("returning nothing")
        
        print("x:", (self.latest_xmin + self.latest_xmax) // 2)
        print("y:", (self.latest_ymin + self.latest_ymax) // 2 )
        self.depth = self.depth_image[(self.latest_ymin + self.latest_ymax) // 2, (self.latest_xmin + self.latest_xmax) // 2]
        print(self.depth)
        if math.isnan(self.depth) or math.isinf(self.depth):
            self.ret = False
        arrow_center = (self.latest_xmin + self.latest_xmax) / 2
        return self.ret, "Cone Detection", arrow_center, self.depth
    
    def arrowdetectmorethan3(self):
        print("Entered process data")
        if self.color_image is None or self.depth_image is None or self.results is None:
            print("returning nothing")
        x = (self.latest_xmin + self.latest_xmax) // 2
        y = (self.latest_ymin + self.latest_ymax) // 2 
        print("x:", x)
        print("y:", y)
        self.depth = self.depth_image[(self.latest_ymin + self.latest_ymax) // 2, (self.latest_xmin + self.latest_xmax) // 2]
        if math.isnan(self.depth) or math.isinf(self.depth) or (x<100 or x>200) or (y<100 or y>200) :
            self.ret = False
        arrow_center = (self.latest_xmin + self.latest_xmax) / 2
        return self.ret, "Not Available", arrow_center, self.depth
    
    def arrowdetectlessthan3(self):
        print("SOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOHHHHHAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAM")
        check = True
        dirn=None
        ret=False
        max_val_l,min_val_l,min_loc_l,max_loc_l = 0.0,0.0,0.0,0.0
        i_max = 0.0
        vals = np.arange(0.03,0.3,0.03)
        z=None
        # framecheck, frame = self.video_feed.read()# Read a single frame
        # if(framecheck==False):
        #     print("meow")
        #     return ret,dirn,None,z
        img = cv.GaussianBlur(self.color_image.copy(),(5,5),3)
        print(f"Image Shape = {img.shape}")
        if img is None:
            print("NO IMAGE FOUNDDDD")
            return ret, dirn, None, z
        self.hp_cam_img2 = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
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
                if float(max_val_l)>=0.71:
                    cv.rectangle(self.hp_cam_img2,top_left, bottom_right, 120, 2)
                max_val_c = max_val_l
            else:
                dirn = "right"
                top_left = max_loc_r
                bottom_right = (top_left[0] + w_finalr, top_left[1] + h_finalr)
                if float(max_val_r)>=0.71:
                    cv.rectangle(self.hp_cam_img2,top_left, bottom_right, 120, 2)
                max_val_c = max_val_l
        else:
            check = False
        # cv.imshow('frame',img2)
        # cv.waitKey(0)

        ## Depth calculation
        if(check):
            if(bottom_right[0] >= img.shape[0] or bottom_right[1] >= img.shape[1]):
                self.ret = False
                z = 2.5
                point = None
            else:
                
                img_height = abs(top_left[1]+bottom_right[1])
                img_width = abs(top_left[0]+bottom_right[0])
                point = (int((img_width) / 2), int((img_height) / 2))
                z = self.depth_image[point[1], point[0]]
                print(f"Soham Distance in else block = {z}")
                if(math.isnan(z) or math.isinf(z)):
                    if not( math.isnan(self.distance) or math.isinf(self.distance)):
                        if(self.distance < 2.0):
                            self.ret = True
                    else:
                        print("Skill Issue")
                        self.ret = False
                    # self.search()
                    z = 2.5
            # img_height = abs(top_left[1]-bottom_right[1])
             
            print(f"Top Left = {top_left}, bottom right = {bottom_right}")
            print(f"Soham Distance = {z}")
            print(point)
            # known_height = 21

            # f = 1117.60839

            # z = (f*known_height)/(img_height)
            conf = (round(float(max_val_c)*100))

            conf_str = str(conf)+"%"

            if(conf>69):
                # cv.putText(self.hp_cam_img2, str(z), (100,100), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                # cv.putText(self.hp_cam_img2, conf_str, (500,100), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                print("Confidence from cv:",conf)
                
                ret=True
                    # cv.putText(self.hp_cam_img2, dirn, (800,100), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                print("Direction:",dirn)
                print(f"Arrow detected at scale: {i_max}")
            else:
                dirn=None
            print("Confidence:", conf)
            cv.imshow('frame',self.hp_cam_img2)
            cv.waitKey(1)
            return ret, dirn, None, z
            
    def cone(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 150, 20), (15, 255, 255))
        kernel = np.ones((5, 5))
        img_thresh_opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        img_thresh_blurred = cv2.medianBlur(img_thresh_opened, 5)
        thresh = img_thresh_blurred
        contours, _ = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        hull = []
        bounding_rect = []
        for c in contours:
            approx = cv2.approxPolyDP(c, 0.025 * cv2.arcLength(c, True), True)
            hull.append(cv2.convexHull(approx))
            bounding_rect = []
            for ch in hull:
                if self.convex_hull_pointing_up(ch):
                    bounding_rect.append(cv2.boundingRect(ch))
                    x, y, w, h = cv2.boundingRect(ch)
                    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        return bounding_rect
    
    def convex_hull_pointing_up(self, ch):
        points_above_center, points_below_center = [], []
        x, y, w, h = cv2.boundingRect(ch)
        aspect_ratio = w / h
        if aspect_ratio < 0.7 and (w * h) > 10000:
            vertical_center = y + h / 2
            for point in ch:
                if point[0][1] < vertical_center:
                    points_above_center.append(point)
                elif point[0][1] >= vertical_center:
                    points_below_center.append(point)
                if len(points_below_center) > 0 and len(points_above_center) > 0:
                    left_x = points_below_center[0][0][0]
                    right_x = points_below_center[0][0][0]
                else:
                    return False
                for point in points_below_center:
                    if point[0][0] < left_x:
                        left_x = point[0][0]
                    if point[0][0] > right_x:
                        right_x = point[0][0]
                for point in points_above_center:
                    if (point[0][0] < left_x) or (point[0][0] > right_x):
                        return False
                else:
                    return False
        return True
    
    def move_straight(self):
        msg = WheelRpm()
        msg.hb = False
        # Aadit's p controller
        print("======MOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE========")
        '''
        if (
            abs(self.initial_drift_angle - self.z_angle) > 20 and not self.searchcalled
        ):  # only if large drift (of 10 degrees) is there, correct it.
                # while it doesn't come back to near the proper orientation, keep giving omega

                msg.omega = int(
                    0
                    + self.kp_straight_rot * (self.initial_drift_angle - self.z_angle)
                )
                msg.omega += 10 * int(
                    (self.initial_drift_angle - self.z_angle)
                    / abs(self.initial_drift_angle - self.z_angle)
                )  # in case it can't rotate fast    # c*x/abs(x) is basically c*(sign of x)

                # capping velocity
                if msg.omega < -self.drift_correction_const:
                    msg.omega = -self.drift_correction_const
                elif msg.omega > self.drift_correction_const:
                    msg.omega = self.drift_correction_const
                #msg.vel = 0
                print("correcting drift with omega =", msg.omega)
                wheelrpm_pub.publish(msg)
                rate.sleep()
            #rospy.sleep(1)
        
        msg.omega = 0
        '''
        if self.init or self.searchcalled:
            print("=====move_straight() is being ignored due to search().======")
            msg.vel = 0
            wheelrpm_pub.publish(msg)
        elif self.ret:
            if abs(self.distance)> self.circle_dist:
                if self.distance != 0.0 and self.distance != self.circle_dist:
                    msg.vel = max(
                        50, int(0 + self.kp * (self.circle_dist - self.distance))
                    )
                    print("Moving straight. ", (self.circle_dist - self.distance))
                    wheelrpm_pub.publish(msg)
                else:
                    print("I'm inside vel = 25")
                    msg.vel = 25
                    wheelrpm_pub.publish(msg)

            else:
                msg.vel = 0
                msg.omega = 0
                wheelrpm_pub.publish(msg)
                print("Stopped going Straight")
                self.gpscalled = 1
                gps_data_pub.publish(self.gpscalled)
                if(self.count_arrow == self.arrow_numbers):
                    self.v1_competition()
            
                if self.direction == "Not Available":
                    self.turn = False
                    

                else:
                    print("I'm in else block of move_straight")
                    lat_sum=0
                    lon_sum=0
                    for i in range(100):
                	    gps_data_pub.publish(self.gpscalled)
                	    lat_sum+=self.current_latitude
                	    lon_sum+= self.current_longitude
                	    print("Sleeping for 10s")
                	    rate.sleep()  # 10s   # Competition rules say 10 s
                    if self.count_arrow <= self.arrow_numbers:
                        lat_av= lat_sum/100
                        lon_av = lon_sum/100
                    	# self.latlong[0].append(msg.latitude)
                	    # self.latlong[1].append(msg.longitude))
                        file_object = open("coordinates.txt", "w")
                        file_object.write("Latitude: " + str(lat_av) +  "Long: " + str(lon_av))
                        file_object.close()
                        with open("sparsh2.csv", "a", newline="") as csv_file:
                    	    csv_writer = csv.writer(csv_file)
                    	    csv_writer.writerow([lat_av, lon_av])
                    	    
                        self.gpscalled = 0
                        print()
                        print("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
                        print("lat:", self.current_latitude)
                        print("long:", self.current_longitude)
                        print()
                        self.pls_call_rot_once=True
                        self.turn = True
                        self.rotate_angle = 90
                        self.initial_yaw = self.z_angle
                        #self.count_arrow += 1
                    else:
                    	self.v1_competition()
                	    # rospy.sleep(10)
	                    #self.write_coordinates()

                    
                    
                        
                   
        else:
            print("Forward")
            msg.vel = 50
            wheelrpm_pub.publish(msg)
            self.turn = False

    def v1_competition(self):
        msg_stop=WheelRpm()
        msg_stop.hb=False
        msg_stop.vel=msg_stop.omega=0
        wheelrpm_pub.publish(msg_stop)
        print("Course completed(hopefully)")
        while not rospy.is_shutdown():
            if(self.state==True):
                print("Press 'A' to go to joystick mode.")
                rate.sleep()

            else:
                print("Rover in Joystick mode")
                rate.sleep()

    def process_dict(self):
        print("self.searchcalled (should print true always):", self.searchcalled)
        if not self.init and self.searchcalled:
            # the first part is not needed, because whenever self.searchcalled is set to true, self.init is set to false
            # actually, this if is only not needed, because in main(), there is an if self.searchcalled(), then process_dict(), which takes care of everything

            print("the dictionary with dist:[enc angles] :- ", self.angles_dict)
            self.searchcalled = False
            max_length_key = 0.0

            try:
                print("Entered Try of proces_dict")
                # if min(self.angles_dict.keys()) == '0.0':
                # del self.angles_dict['0.0']
                if min(self.angles_dict.keys()) != 0:
                    print("Entered if")
                    # del self.angles_dict['0.0']
                    max_length_key = min(self.angles_dict.keys())
                    # max_length_key = max(self.angles_dict, key=lambda k: len(self.angles_dict[k]))
                    self.min_dist = max_length_key
            except:
                print("The list is empty. No minimum value.")
                self.init = False
                self.searchcalled = False
                return
            if len(self.angles_dict[max_length_key + 1]) != 0:
                self.which_enc_angle_to_turn = (
                    sum(self.angles_dict[self.min_dist])
                    + sum(self.angles_dict[self.min_dist + 1])
                ) / (
                    len(self.angles_dict[self.min_dist])
                    + len(self.angles_dict[self.min_dist + 1])
                )
            else:
                self.which_enc_angle_to_turn = sum(
                    self.angles_dict[self.min_dist]
                ) / len(self.angles_dict[self.min_dist])
            print("Angle to turn:", self.which_enc_angle_to_turn)
            print("the dictionary with dist:[enc angles] :- ", self.angles_dict)

            # encoder need not be perfect. if in case there is some cup, edit this angle as per your needs
            # editing the below line due to encoder sign being flipped
            #if self.which_enc_angle_to_turn < 0:
            if self.which_enc_angle_to_turn >0:
                self.direction = "left"
                # self.rotate_angle=abs(self.which_enc_angle_to_turn + 0.5*self.angle_thresh) #+2 degrees
                self.rotate_angle = abs(self.which_enc_angle_to_turn)
            else:
                self.direction = "right"
                # self.rotate_angle=(self.which_enc_angle_to_turn - 0.5*self.angle_thresh) #+2 degrees
                self.rotate_angle = abs(self.which_enc_angle_to_turn)

            self.turn = True
            self.initial_yaw = self.z_angle
            self.angles_dict = defaultdict(list)

            # self.init = False
    def rotate(self, dir):
        print("Rotation has been called")
        msg = WheelRpm()
        msg.vel = 0
        msg.hb = False
        diff = self.z_angle - self.initial_yaw
        if diff > 120:
            diff = diff - 360
        elif diff < -120:
            diff = diff + 360
        print("diff=", diff)
        """
        if (-60<self.z_angle-self.initial_yaw<60):
            error=30
        else:
            error = 90-abs(self.z_angle-self.initial_yaw)
        """
        print("Rotation angle:", self.rotate_angle)
        print("Initial Yaw:", self.initial_yaw)
        print("Current z angle", self.z_angle)
        error = self.rotate_angle - abs(diff)
        print("error=", error)
        # if self.direction == -1:
        #   self.rotate_angle = self.rotate_angle +2
        if abs(error) >= 0.5 * self.angle_thresh:
            msg.omega = 0 + (dir * 80)
            msg.vel = 0
            print("Calling Rotate, printing Z angle below")
            print(error)
            wheelrpm_pub.publish(msg)
        else:
            msg.omega = 0
            msg.vel = 0
            wheelrpm_pub.publish(msg)
            # self.z_angle=0

            self.initial_drift_angle = self.z_angle
            print("****ROTATE DONE*****")
            # self.distance = 10.0
            self.start_time = time.time() - 10
            self.initial_yaw = self.z_angle
            self.turn = False
            self.direction = "Not Available"
            #rospy.sleep(2)
    
    def rot_in_place(self, di):
        print("ROVER STOPPED ---------- Rot-in-place has been called")
        msg = WheelRpm()
        msg.omega=0
        wheelrpm_pub.publish(msg)
        # msg.hb = True
        #print(self.pls_call_rot_once)
        if self.pls_call_rot_once:
            self.rot = 1
            rotin_pub.publish(self.rot)
            self.pls_call_rot_once=False

        print("time.time():", time.time())
        print("self.start_time:", self.start_time)
        # if time.time() - self.start_time < self.time_thresh_rot:  # time_thresh is 5s
        #     print(
        #         "time.time()-self.start_time (when this becomes 5s, rotinplace will end):",
        #         time.time() - self.start_time,
        #     )
        
        diff = self.z_angle - self.initial_yaw
        if diff > 120:
            diff = diff - 360
        elif diff < -120:
            diff = diff + 360
        print("diff=", diff)
        """
        if (-60<self.z_angle-self.initial_yaw<60):
            error=30
        else:
            error = 90-abs(self.z_angle-self.initial_yaw)
        """
        print("Rotation angle:", self.rotate_angle)
        print("Initial Yaw:", self.initial_yaw)
        print("Current z angle", self.z_angle)
        error = self.rotate_angle - abs(diff)
        print("error=", error)

        # if self.direction == -1:
        #   self.rotate_angle = self.rotate_angle +2
        if abs(error) >= 0.5 * self.angle_thresh:
            msg.omega = 0 + (di * 40)
            #msg.vel = di*25 
            print(di)
            print("Calling ROT-IN-PLACE, printing Z angle below")
            print(error)
            wheelrpm_pub.publish(msg)
        else:
            msg.omega = 0
            msg.vel = 0
            msg.hb = False
            wheelrpm_pub.publish(msg)

            self.rot = 2
            rotin_pub.publish(self.rot)
            rospy.sleep(5)
            self.rot =0
            rotin_pub.publish(self.rot)
            # self.z_angle=0

            self.initial_drift_angle = self.z_angle
            print("****ROTATE DONE*****")
            self.count_arrow +=1
            # self.distance = 10.0
            self.start_time = time.time() - 10
            self.initial_yaw = self.z_angle
            self.turn = False
            self.direction = "Not Available"
            #rospy.sleep(2)

    def search(self):
        print("==========================================================================================================SEARCH START=====================================================================================")
        if time.time() - self.start_time < self.time_thresh:  # time_thresh is 20s
            print(
                "time.time()-self.start_time (when this becomes 20s, search will happen):",
                time.time() - self.start_time,
            )
            return
        else :
            self.time_bool =True
        print("self.searchcalled ", self.searchcalled)
        #if abs(self.enc_data) < 0.6 * self.angle_thresh and self.ret and not self.init and self.time_bool:
            # if arrow is detected and realsense is facing straight, then come out of search immediately.
        #    return

        print("Search() has been called.")
        self.searchcalled = True
        
        print("time.time():", time.time())
        print("self.start_time:", self.start_time)
        msg1 = WheelRpm()
        msg1.hb = False
        msg1.omega = 0
        msg1.vel = 0
        wheelrpm_pub.publish(msg1)
        print("Rover has stopped.")
        msg = std_msgs.Int32MultiArray()
        msg.data = [0, 0, 0, 0, 0, 0]

        msg.layout = std_msgs.MultiArrayLayout()
        msg.layout.data_offset = 0

        msg.layout.dim = [std_msgs.MultiArrayDimension()]
        msg.layout.dim[0].size = msg.layout.dim[0].stride = len(msg.data)
        msg.layout.dim[0].label = "write"
        self.pub.publish(msg)

        print("Entered while loop.")
        while (
     
            self.init == False
            and abs(self.enc_data) < abs(self.start_angle) - 2 * self.angle_thresh
        ):
            # to make the realsense go to the 60 degree maximum before starting the burst search
            print(self.enc_data)
            print("meaoo")
            msg.data = [0, 0, 0,0,0,0]
            msg.data[self.base_index]=255*self.base_rot_dir
            rate.sleep()
            self.pub.publish(msg)
            #self.start_time = time.time() - self.time_thresh
        msg.data = [0, 0, 0, 0, 0, 0]
        print("Exited while loop.")
        self.init = True
        print("self.init (set to true in the previous line:", self.init)
        print("Realsense's angle:", self.enc_data)
        print("self.ret:", self.ret)
        if (
            self.init == True
            and abs(self.enc_data) <= self.start_angle
            and not self.ret
        ):
            # if arrow is not detected and the realsense has not gone beyond the 60 degree maximum, continue moving realsense
            # self.init is not made false, so this will happen again when search is called in main()
            print("self.enc_data: ", self.enc_data)
            print("self.start_angle: ", self.start_angle)
            print("Camera Moving")
            #self.ret, direction, pix, self.distance = self.arrowdetectmorethan3()
            msg.data = [0,0,0,0,0, 0]
            msg.data[self.base_index]=-255*self.base_rot_dir
            rate.sleep()
            self.pub.publish(msg)
            print("deistance from search:XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX ",self.distance)
            # main area
        elif (
            self.init == True
            and abs(self.enc_data) <= self.start_angle
            and self.ret
        ):
            # if arrow is detected and the realsense is within the 60 degree maximum, append the arrow's values and continuemoving realsense
            # self.init is not made false, so this will happen again when search is called in main()

            # self.distance = self.distance*1
            # self.distance = self.distance // 1
            # self.distance = self.distance / 1
            if not(math.isnan(self.distance) or math.isinf(self.distance)):
                self.distance = float(round(self.distance))
            if self.distance > 0.0:  # change
                self.angles_dict[self.distance].append(self.enc_data)
                print(f"I'm appending to angles_dict at angle = {self.enc_data} and distance = {self.distance}")
            msg.data = [0,0, 0, 0, 0, 0]
            msg.data[self.base_index]=-255*self.base_rot_dir
            self.pub.publish(msg)
            rate.sleep()
            print("Arrow found at: ", self.enc_data)
            print()
        elif (abs(self.enc_data) >= self.start_angle) :
            # when the realsense has crossed the 60 degree maximum, realsense comes back to middle and the dictionary is processed
            # self.init is set to false (when next search() is called, realsense will first move to the 60 degree maximum)
            # and the counting of time is reset (that is, the next search will happen at least 20s after this block of code)

            while abs(self.enc_data) > self.angle_thresh:
                if self.enc_data > 0:
                    msg.data = [0, 0,0,0, 0, 0]
                    msg.data[self.base_index]=255*self.base_rot_dir
                else:
                    msg.data = [0,0, 0,0,0, 0]
                    msg.data[self.base_index]=-255*self.base_rot_dir
                rate.sleep()
                self.pub.publish(msg)
                self.start_time=time.time()
            msg.data = [0, 0, 0, 0, 0, 0]
            self.pub.publish(msg)
            self.init = False
            self.searchcalled = True
            self.distance = 10.0
            #self.start_time = time.time()
        """     
            while self.enc_data >5:
                pass
                #go in one direction to 0.
            while self.enc_data < -5:
                pass
                #go in other direction to 0.
            return
        """
        print("==========================================================================================================SEARCH END=====================================================================================")
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

    def gps_callback(self, msg):
        if msg.latitude and msg.longitude:
            self.current_latitude = msg.latitude
            self.current_longitude = msg.longitude

    def main(self):
        gps_data_pub.publish(self.gpscalled)
        rotin_pub.publish(self.rot)
        print("Pls call rot:",self.pls_call_rot_once)
        print("self.rot:", self.rot)

        if (
            self.count_arrow == self.arrow_numbers
        ):  # change before competition  #if we take intervention, change this number acc to your needs
            print("Im in if block of main")
            if (not self.turn and not self.ret) or (self.init):
                self.search()
                if self.searchcalled:
                    self.process_dict()

            if not self.turn:
                self.ret, direction, pix, self.distance = self.cone_model()
                if self.ret == True:
                    print("Cone detected")
                    print("self.cone_distance:", self.distance)

                else:
                    print("Still searching")
                    self.ret = False
                self.move_straight()
            else:  # if self.turn is true, everything stops and rover does only turning
                print("Im going into rotate block in main")
                
                if self.direction == "left":
                    print("rotating left")
                    self.rotate(1)
                else:
                    print("rotating right")
                    self.rotate(-1)

        else:
            print("Im in else block of main")
            if (not self.turn and not self.ret) or (self.init):
                self.search()
                print("self.distance:", self.distance)
                if self.searchcalled:
                    self.process_dict()
            if (
                not self.turn
            ):  # this is there because detection need not happen when turning
                # note that self.turn is made true once in process_dict(), so we put this if condition again.

                self.ret, direction, pix, self.distance = self.arrowdetectmorethan3()
                #Uncomment later when testing outside
                print(f"Depth = {self.distance}, center = {pix}")
                #self.distance = 1.5
                # I've changed 3.0 to 2.0
                if self.distance < 1.5 and not self.searchcalled:
                    #ret, depth_frame, color_frame, depth = self.get_frame()
                    ret, direction, pix, distance = self.arrowdetectlessthan3()
                    # if not self.ret:
                    #     self.ret,self.direction,pix,self.distance=self.arrowdetectmorethan3()
                    if ret == True:
                        self.ret = ret
                        self.direction = direction
                        self.distance = distance
                        

                if self.ret:
                    print("arrow detected at distance: " + str(self.distance))
                    print("Direction: " + self.direction)
                else:
                    print("Trying to detect arrow...")
                self.move_straight()

            else:  # if self.turn is true, everything stops and rover does only turning
                print("Im going into rotate block in main")
                if self.distance>1.5:
                    if self.direction == "left":
                        print("rotating left")
                        self.rotate(1)
                    else:
                        print("rotating right")
                        self.rotate(-1)
                else:
                    if self.direction == "left":
                        print("rotating left")
                        self.rot_in_place(-1)
                    else:
                        print("rotating right")
                        self.rot_in_place(1)
                        
              
    def run(self):
        # Main loop
        rate = rospy.Rate(10) 
        while not rospy.is_shutdown():
            print("goin in")
            if(self.image_avbl):
                if(self.count_arrow < self.arrow_numbers):
                    self.get_box()
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


