#!/usr/bin/env python3

import sys
import rospy
import math
import time
import cv2
import numpy as np
import imutils
from geometry_msgs.msg import Twist, Point
from traversal.msg import WheelRpm
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator
import open3d as o3d
from threading import Lock, Thread
from time import sleep
from nav_msgs.msg import Odometry
import torch
import argparse
# import cv_viewer.tracking_viewer as cv_viewer
from collections import defaultdict
# from iroc_center import find_center
import tf
# from tf.transformations import quaternion_from_euler, euler_from_quaternion
from tf import LookupException, ConnectivityException, ExtrapolationException
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Imu


# three way points before 1st goal
# consider as 3 goals only - did
# once you cross goal get some counter - did

class habibo():
    def __init__(self):

        # self.initial_pitch = None
        # self.initial_roll = None
        self.initial_yaw = 0
        # self.start_time = time.time()
        self.initial_odom = [0, 0]
        self.odom_initialized = False

        self.model = YOLO(
            "/home/nvidia/galileo2024/src/navigation2/scripts/model.pt")
        self.cv_image = []
        self.current_pitch = self.current_roll = self.current_yaw = 0.0	
        self.initial_pitch = self.initial_roll = self.initial_yaw = 0.0
        # get waypoint coordinates and put it in here
        #self.goals_x=[0.97,2.44,3.3,3.4,8.41]
        #self.goals_y=[0.97,-1.71,-1.9,-0.77,-3.0]
        self.goals_x = [0.9, 2.45, 2.45, 5.2, 6.22]
        self.goals_y = [1.0, 1.8, 1.5, 2.2, 1.55]
        # self.goals_x[2] = float(input("Enter the x1 coordinate: "))
        # self.goals_y[2] = float(input("Enter the y1 coordinate: "))

        # self.goals_x[3] = float(input("Enter the x2 coordinate: "))
        # self.goals_y[3] = float(input("Enter the y2 coordinate: "))
        self.bridge = CvBridge()

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_callback)
        # rospy.Subscriber("/position_wrt_map", Float32, self.map_callback)
        # rospy.Subscriber("/zed2i/zed_node/left/image_rect_color", Image, self.image_callback)
        rospy.Subscriber("/zed2i/zed_node/rgb/image_rect_color",
                         Image, self.image_callback)
        rospy.Subscriber("/zed2i/zed_node/depth/depth_registered",
                         Image, self.depth_callback)
        rospy.Subscriber("/zed2i/zed_node/imu/data", Imu, self.imu_callback)
        #rospy.Subscriber("zed2i/zed_node/left/image_rect_color",
         #	                Image, self.callback)
        #rospy.Subscriber("/ik_over_ah", Bool, self.ik_dynamic_callback)
        # self.initial_pitch = self.current_pitch*180/3.14
        # self.initial_roll = self.current_roll*180/3.14
        # self.initial_yaw = self.current_yaw*180/3.14

        self.omega = 0
        self.pixel_dict = defaultdict(list)
        self.turn_completed = False  # to keep track of the initial turning
        self.obs_avd_completed = False
        self.goal_counter =0
        self.kutty_obstacle = False
        self.ignore_everything = False
        self.p_l, self.p_r, self.p_u, self.p_d = 0, 0, 0, 0
        self.tube_pose_pub = rospy.Publisher("tube_pose", Point, queue_size=10)
        self.arm_pub = rospy.Publisher(
            'arm_goal', Float32MultiArray, queue_size=10)
        self.bool_pub = rospy.Publisher('arm_goal_bool', Bool, queue_size=10)

        # self.velocity_pub = rospy.Publisher('motion',  WheelRpm, queue_size = 10)
        self.is_identified = False
        self.depth = None
        self.ik_over_ah = 0

        #  if err != sl.ERROR_CODE.SUCCESS:
        #     print(repr(err))
        #    self.zed.close()
        #   exit(1)
        # brightness_value = 0
        # self.bridge = CvBridge()

        # self.odom_self = np.array(2)
        self.odom_self = [0.0, 0.0]
        # if self.goal_counter == 0:
        # 	self.goal_x = self.goals_x[0]
        # 	self.goal_y = self.goals_y[0]
        self.desired_angle = 0.0
        self.ret_cube = False  # bool for saying if crater is there
        self.ret_crater = False
        #self.tanish = False
        self.image_arrived = False
        self.p_x, self.p_y = 0, 0
        self.main_control = False
        self.desired_angle_gtg = 0.0
        self.ret_guiding_cube = False
        self.ret_guiding_cube_override = True
        self.snatch_control = True
        #self.anuj = False
        #self.pranav = False
        #self.ik_over_ah = False
        #self.ik =1

        # self.zed.set_camera_settings(sl.VIDEO_SETTINGS.BRIGHTNESS, brightness_value)
        time.sleep(2)

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z  # in radians

    def find_height(self, top, bottom, left, right, depth):
        # run continuously until find next
        f_length = 2.1
        print("im inside find_height")
        # sensor_height = 500  # Height of the camera sensor in pixels
        object_height = top - bottom  # Object height in pixels
        #print("object_height", object_height)
        pixel_height = abs(object_height)

        # self.zed camera provides depth in millimeters
        # Depth at the center of the bounding box
        depth_value = self.depth_image[int(
            (top + bottom) / 2), int((left+right)/2)]
        
        #print("depth_value inside find_height", depth_value)
        # print(depth_value)
        # depth_value = depth_value[1]
        # print(depth_value)
        # # Convert depth value from millimeters to meters
        # depth_meters = depth_value / 1000.0

        # Calculate the height of the object using similar triangles
        height = pixel_height*depth_value/(f_length*1.3)

        return height

    # yolo parts

    def cube_detect_YOLO(self):
        # here there should be an edit about how to change or wait acc to requirement. Change n here
        print("Detection")

        self.ret_obstacle = False

        # self.depth_obstacle = []
        self.depth_obstacle = 0.0

        if self.cv_image is not None:
            print("image is not none")
            img = self.cv_image
            print("Dimensions of self.cv_image", np.shape(self.cv_image))
            results = self.model.predict(img, conf=0.45, max_det=3)
            self.ret_guiding_cube = False
            if len(results)== 0:
                self.ret_guiding_cube = False
                self.ret_guiding_cube_override = True

            for r in results:
                # Ensure image is contiguous
                annotator = Annotator(np.ascontiguousarray(img))
                boxes = r.boxes
                for box in boxes:
                    # get box coordinates in (left, top, right, bottom) format
                    b = box.xyxy[0]
                    c = box.cls
                    annotator.box_label(b, self.model.names[int(c)])
                    # Draw rectangle around the object
                    left, top, right, bottom = map(int, b)
                    cv2.rectangle(img, (left, top),
                                  (right, bottom), (0, 255, 0), 2)
                    try:
                        if (left+right)/2 < 540 or (left + right)/2 > 100:
                            depth_value = self.depth_image[int(
                                (top+bottom)/2), int((left+right)/2)]
                            print("depth_value: ", depth_value)
                            # self.depth_obstacle.append(depth_value)
                            self.depth_obstacle = depth_value
                            height = self.find_height(
                                top, bottom, left, right, depth_value)

                            print('top, bottom,left,right,depth_value',
                                    top, bottom, left, right, depth_value)
                            print("Height of the object:", height)
                            if height < 24 and depth_value<1 and depth_value > 0.8:
                                if abs(self.current_yaw - self.desired_angle_gtg) < 6:
                                    #self.ret_guiding_cube = True
                                    #self.safe_obstacle_avoidance(left, right, depth_value)
                                    # x, y, z = self.posn_transformation(
                                        #left, right)
                                    print("checku")
                                    #if abs(x - self.goals_x[self.goal_counter]) < 0.3 and abs(y - self.goals_y[self.goal_counter]) < 0.3:
                                        #  sel    f.ignore_everything = True
                                        # self.p_l, self.p_r, self.p_u, self.p_d = left, right, top, bottom 
                            

                            elif height > 24 and height <40 and depth_value < 1.2 and self.goal_counter >=3:
                                self.ret_obstacle = True
                                print("self.ret_obstacle", self.ret_obstacle)
                                self.obstacle_avoidance(left, right)
                                rospy.sleep(0.25)
                    #if int(c) == 2 and self.goal_counter == 1:
                            # self.tanish
                            # self.tanish = True
                            #if (left+right)/2 < 640 and (left+right)/2 > 60:
                                #depth_value = self.depth_image[int(
                                 #   (top+bottom)/2), int((left+right)/2)]
                                #print("depth_value of cylinder", depth_value)
                                #if depth_value <= 1:
                                    #self.tanish == True
                                    #print("self.tanish", self.tanish)
                        #if int(c) == 3 and self.goal_counter == 2:
                         #   self.anuj = True
                            # not important rn. tc of this later

                    except Exception as e:
                        print("Error:", e)

                # Show annotated image
            #cv2.imshow('YOLO V8 Detection', img)
            #if cv2.waitKey(1) & 0xFF == ord('q'):
              #	  return

    def posn_transformation(self, p_x, p_y):
        # K matrix values
        f_x = 527.2972398956961
        f_y = 527.2972398956961
        c_x = 658.8206787109375
        c_y = 372.25787353515625
        cord_x = (p_x-c_x)/f_x
        cord_y = (p_y-c_y)/f_y
# print("sum and len of depth_obstacle", sum(self.depth_obstacle), len(self.depth_obstacle))
# cord_z=sum(self.depth_obstacle)/len(self.depth_obstacle)
        cord_z = self.depth_obstacle
        return cord_x, cord_y, cord_z

    def obstacle_avoidance(self, x, y):
        x, y, z = self.posn_transformation(x, y)
        # obs_vec = np.array(2)
        obs_vec = [x, y]
        kp = 1.0                # proportionality constant. set to 1 for now
        print("Im inside obstacle_avoidance now")
        # negate_obs = np.array(2)
        negate_obs = [-x, -y]
        #print("negate_obs", negate_obs)
        # goal_vec = np.array(2)
        goal_vec = [self.goals_x[self.goal_counter],
                    self.goals_y[self.goal_counter]]
        unit_nobs = []
        # unit_nobs.append(negate_obs[0]/(x**2 + y**2))
        unit_nobs.append(negate_obs[1]/(0.1*(x**2 + y**2)))
        unit_nobs.append(negate_obs[0]/(0.1*(x**2 + y**2)))
        #print("unit_nobs", unit_nobs)
        # heading_vec = np.array(2)
        heading_vec = [goal_vec[0] + unit_nobs[0], goal_vec[1] + unit_nobs[1]]
        #print("heading_vec", heading_vec)
        desired_turn = math.atan(heading_vec[1]/heading_vec[0])*360/math.pi
        #print(f"Desired turn in obstcl avd = {desired_turn}")
        g = WheelRpm()
        self.omega = int(kp*(self.current_yaw - desired_turn))
        g.vel = 0
        g.omega= 0
        #print("desired_turn - self.current_yaw in obstacle avoidance",
              #desired_turn - self.current_yaw)
        if abs(desired_turn - self.current_yaw) > 5:
            print("self.omega", self.omega)

            if self.omega > 0:
                g.omega = -20
            else:
                g.omega = -20
        else:
            print("self.omega", self.omega)
            g.omega = 0
        self.obs_avd_completed = True
        vel_pub.publish(g)
        self.turn_completed = False

# rospy.sleep(0.25)
        return heading_vec
    
    def safe_obstacle_avoidance(self,l,r,d):
        g = WheelRpm()
        kp = 10

        kp_rot = 0.25
        self.snatch_control = True
        print("pixel_values_of_guiding_cube", (l+r)/2)
        if (l+r)/2 <370 and (l+r)/2 > 270 :
            g.vel = int(kp * d)
        elif (l+r)/2 >370 :
            g.omega = int(min(10,kp_rot * (320-(l+r)/2)))
            g.vel = 0
        elif (l+r)/2 <270:
            g.omega = int(max(-10, kp_rot*(320-(l+r)/2)))
            g.vel = 0
        else:
            self.ret_guiding_cube_override = True
        if(self.ret_guiding_cube_override == False):
            vel_pub.publish(g)
            self.snatch_control = False
            print("what is being published in safe_obstacle_avoidance?", g)

            


    def show_coordinates(self, p_x, p_y, cv_image):
        # K matrix values
        f_x = 527.2972398956961
        f_y = 527.2972398956961
        c_x = 300.0
        c_y = 180.0
        self.cord_x = self.depth*(p_x-c_x)/f_x
        self.cord_y = self.depth*(p_y-c_y)/f_y
        self.cord_z = self.depth
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = 0.5
        self.color = [255, 0, 0]
#self.current_yaw - desired_turn 135.73556174915615
        self.thickness = 1


    def go_to_goal(self, n):
        if self.goal_counter == 3 and self.ik_over_ah ==0:
            print("inside 3")
            g = WheelRpm()
            g.vel = 0
            g.omega = 0
            vel_pub.publish(g)
            ik_pick_up=int(input("is ik over?press 1 for yes and 0 for no"))
            #ik_pick_up =
            if ik_pick_up==1:
                self.ik_over_ah+=1
                mini1 = time.time()

                while time.time()-mini1< 6: 
                    print("going back")
                    g.vel = -20
                    g.omega = 0
                    vel_pub.publish(g)
            else:
                self.ik_over_ah=0
        elif self.goal_counter==4 and self.ik_over_ah==1:
            print("inside 4")
            g = WheelRpm()
            g.vel = 0
            g.omega = 0
            vel_pub.publish(g)
            ik_drop=int(input("is tube dropped? press 1 for yes,0 for no"))
            if ik_drop==1:
                self.ik_over_ah+=1
            else:
                self.ik_over_ah=1
        elif self.goal_counter==4 and self.ik_over_ah==2:
            print("going backwards")
            g=WheelRpm()
            g.vel=-20
            g.omega = 0
            mini =time.time()
            while time.time()- mini < 6:
                vel_pub.publish(g)
                rate.sleep()
            if time.time() - mini > 6:
                g.vel = 0
                g.omega = 0
                vel_pub.publish(g)
                self.ik_over_ah = 3
                print("self.ik_over_ah in for loop", self.ik_over_ah)

            	
        elif self.ik_over_ah == 3:
            print("mission completed")
            g = WheelRpm()
            g.vel = 0
            g.omega = 0
            vel_pub.publish(g)
            
        else: # (self.goal_control == True and self.ik_over_ah:
            #main_pub.publish(False)
            # waypoint navigation within this
            kp = 10
            kp_rot = 0.8
            kp_linear = 25
            # goal_vec = np.array(2)
            # print("goal_vec:",goal_vec)
            goal_vec = [self.goals_x[n], self.goals_y[n]]
            print("goal_vec: ", goal_vec)
            self.desired_angle_gtg = math.atan(
                (goal_vec[1]-self.odom_self[1])/(goal_vec[0]-self.odom_self[0]))
            self.desired_angle_gtg = (self.desired_angle_gtg)*180/math.pi
            print("desired turn in gtg", self.desired_angle_gtg)
            goal_distance = math.sqrt(
                ((goal_vec[0] - self.odom_self[0])**2) + (goal_vec[1] - self.odom_self[1])**2)
            print("goal distance in metres", goal_distance)
            print("self.goal_counter", self.goal_counter)
            self.omega = int(kp*(self.current_yaw - self.desired_angle_gtg))
            desired_vel = int(kp_linear * goal_distance)
            g = WheelRpm()
            g.vel = min(desired_vel, 13)
            
            if(goal_distance < 0.5):
                self.ret_guiding_cube_override = True

            if goal_distance < 0.25:
                g.vel = 0
                g.omega = 0
                self.goal_counter += 1
                self.ret_guiding_cube_override = False

            if self.obs_avd_completed == True:
                print("self.obs_avd_completed", self.obs_avd_completed)
                mini = time.time()
                while time.time()-  mini <6:
                    g.omega = -15
                    g.vel = 0
                    vel_pub.publish(g)
                g.omega = 0
                g.vel = 13
                vel_pub.publish(g)
                rospy.sleep(2)
                self.obs_avd_completed = False
            else:
                print("self.current_yaw - desired_turn",
                    self.current_yaw - self.desired_angle_gtg)
                if abs(self.current_yaw - self.desired_angle_gtg) > 6:
                    print("self.omega gtg", self.omega)
                    if self.omega > 0:
                        g.omega = int(max(15,int(min(kp_rot*(self.current_yaw - self.desired_angle_gtg), 20))))
                        g.vel = 0
                    else:
                        g.omega = int(min(-15,int(max(kp_rot*(self.current_yaw - self.desired_angle_gtg), -20))))
                        g.vel = 0
                elif abs(self.current_yaw - self.desired_angle_gtg) < 6:
                    print("self.omega gtg", 0)
                    g.omega = 0
            if(self.ret_guiding_cube == False) or (self.ret_guiding_cube_override == True) or  self.snatch_control == True:
                vel_pub.publish(g)
                print("Publishing velocity/omega from GTG", g)
                
    def main(self):
        #if self.main_control == False:
        self.cube_detect_YOLO()
		# cube_thread.start()
		#print("check1 main")
        print("self.odom = ", self.odom_self)
        print("self.orientation", self.current_yaw)
        print("initial_orientation", self.initial_yaw)
		# rospy.sleep(1)
		# self.crater_detect_YOLO()
		# rospy.sleep(1)
		#        crater_thread.start()
        print("self.guidingbox: ", self.ret_guiding_cube)
        print("self.snatch_control", self.snatch_control)
		# n = self.goal_counter
        if self.ret_obstacle == False :  # :
            self.go_to_goal(self.goal_counter)
            #print("end of main")

		
    # callbacks

    def odom_callback(self, data):
        self.odom_self[0] = data.pose.pose.position.x
        self.odom_self[1] = -data.pose.pose.position.y
        if self.odom_initialized == False:
            self.initial_odom[0] = self.odom_self[0]
            self.initial_odom[1] = self.odom_self[1]
            self.odom_initialized = True
        self.odom_self[0] = self.odom_self[0] - self.initial_odom[0]
        self.odom_self[1] = self.odom_self[1] - self.initial_odom[1]
        # self.odom_self[2] = data.z

    def imu_callback(self, data):
        current_x = data.orientation.x
        current_y = data.orientation.y
        current_z = data.orientation.z
        current_w = data.orientation.w
        self.current_pitch, self.current_roll, self.current_yaw = self.euler_from_quaternion(
            current_x, current_y, current_z, current_w)

        if self.initial_yaw == 0:
            # self.current_pitch = self.current_pitch*180/3.14 - self.initial_pitch
            # self.current_roll = self.current_roll*180/3.14 - self.initial_roll
            self.initial_yaw = self.current_yaw*180/math.pi
        self.current_yaw = -(self.current_yaw * 180/math.pi) + self.initial_yaw
        if self.current_yaw < -120:
            self.current_yaw = self.current_yaw + 360
        elif self.current_yaw > 120:
            self.current_yaw = self.current_yaw - 360


    def image_callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            print(e)

    def depth_callback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
        except Exception as e:
            print(e)

    def ik_dynamic_callback(self, data):
        self.ik_over_ah = data
        print("inside ik_over_ah callback")
        if self.ik_over_ah.data == True:
            self.main_control = False
            main_pub.publish(False)
    

    def spin(self):
        while not rospy.is_shutdown():
            self.main()
            rate.sleep()



if __name__ == "__main__":
    try:
        # get goal location. I have no clue how to provide goal location and stuff
        # not anymore bruh
        rospy.init_node("hihiih")
        vel_pub = rospy.Publisher("motion", WheelRpm, queue_size=10)
        ik_start = rospy.Publisher("ik_start", Bool, queue_size=10)
        ik_start_drop = rospy.Publisher("ik_start_drop",Bool,queue_size=10)
        main_pub = rospy.Publisher("main_control", Bool, queue_size = 10)
        rate = rospy.Rate(20)
        run = habibo()
        run.spin()
    except KeyboardInterrupt:
        sys.exit()


# ID Type X, Y, Z (meters)
# 1 A1 1.6, -0.9, 0.0
# 2 A1 1.4, -1.7, 0.0
# 3 A1 2.5, -1.4, 0.0
# 4 A1 3.0, -2.3, 0.0
# 5 A1 4.7, -1.8, 0.0
# 6 A1 7.58, -2.7, 0.0
# 7 A2 1.2, -2.5, 0.0
# 8 A2 2.0, -0.6, 0.0
# 9 A2 3.0, -0.5, 0.0
# 10 A2 4.5, -1.25, 0.0
# 11 A2 7.38, -1.0, 0.0
# 12 B1 2.3, -2.2, 0.0
# 13 B1 4.7, -2.4, 0.0
# 14 B1 7.58, -1.8, 0.0
# 15 B2 2.7, -0.7, 0.0
# 16 B2 4.5, -3.0, 0.0
# 17 B2 8.58, -0.9, 0.0
# SP Start Position 0, -1.2, 0
# T Sample Tube 3.3, -0.825, 0
# WP Way Point 2.9, -2.1, 0
# C & FP Sample Container & Final Position 9.25, -3.25, 0)
