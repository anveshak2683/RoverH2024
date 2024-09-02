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
from sensor_msgs.msg import LaserScan
from navigation2.msg import red



# listen to tanish what he's giving about red color
# if red na you have to go towards that
# then change odom to goal and work after that

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

        rospy.Subscriber("/zed2i/zed_node/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/zed2i/zed_node/rgb/image_rect_color",
                         Image, self.image_callback)
        rospy.Subscriber("/zed2i/zed_node/depth/depth_registered",
                         Image, self.depth_callback)
        rospy.Subscriber("/zed2i/zed_node/imu/data", Imu, self.imu_callback)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
     #   rospy.Subscriber('/red_scan', Bool, self.tanish_callback)
        rospy.Subscriber('/tanish_commands', red, self.tanish_callback)

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
        self.front_min = 0
        self.front_turn = 0
        self.red_detected = False
        self.tanish_vel = self.tanish_omega = 0
        self.left_check = self.right_check = self.front_check = 0
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
    # yolo parts

    def obstacle_avoidance(self):
        thresh = 2
        msg = WheelRpm()
        direction = 0
        self.ret_obstacle = True
        if self.left_check < thresh:
            direction = 1
        elif self.right_check < thresh:
            direction = -1
        elif self.front_check < thresh:
            if self.front_turn ==1 :
                direction = 1
            elif self.front_turn == -1:
                direction = -1
            else:
                direction = 0
        else:
            self.ret_obstacle = False
        msg.vel = 20
        msg.omega = 20 * direction
        ##################################### Im not publishing anything here for now
        vel_pub.publish(msg)

    
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
            
            if goal_distance < 0.25:
                g.vel = 0
                g.omega = 0
                self.goal_counter += 1
            
           
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
            if (self.ret_guiding_cube_override == True) or  self.snatch_control == True or self.red_detected == False:
                vel_pub.publish(g)
                print("Publishing velocity/omega from GTG", g)
            else:
                g.omega = self.tanish_omega
                g.vel = self.tanish_vel
                vel_pub.publish(g)
                print("Publishing velocity from tanish", g)
                
    def main(self):

        print("self.odom = ", self.odom_self)
        print("self.orientation", self.current_yaw)
        print("initial_orientation", self.initial_yaw)
        self.obstacle_avoidance()
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
    
    def lidar_callback(self, data):
        tmp = np.nan_to_num(data.ranges, copy = True, nan = 0.8, posinf = 1000, neginf = 10000)
        front_check_sum = 0
        left_check_sum = 0
        right_check_sum = 0
        length = len(data.ranges)
        left_start = int(0.25*length)
        right_start = int(0.75*length)
        front_start = int(0.45*length)
        front_min_1 = 100

        for i in range(50):
            left_check_sum = left_check_sum + data.ranges[left_start + i]
            right_check_sum = right_check_sum + data.ranges[right_start - i]
            front_check_sum = front_check_sum + data.ranges[front_start + i]
            if front_min_1 > data.ranges[front_start + i]:
                front_min_1 = data.ranges[front_start + i]
                self.front_min = front_start + i
        if self.front_min < 0.5 * length:
            self.front_turn = 1
        elif self.front_min > 0.5 * length:
            self.front_turn = -1
        self.left_check = left_check_sum / 50
        self.right_check = right_check_sum / 50
        self.front_check = front_check_sum / 50
    
    def tanish_callback(self, data):
        self.red_detected = data.detect
        self.tanish_vel = data.velfromtanish
        self.tanish_omega = data.omegafromtanish
    

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
