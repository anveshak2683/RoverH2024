#!/usr/bin/env python3
import sys
import rospy
from navigation.msg import gps_data
from geometry_msgs.msg import Twist
import math
import time
import cv2
import numpy as np
import imutils
from traversal.msg import WheelRpm
from traversal.srv import *
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
import pyrealsense2 as rs
import threading
import std_msgs.msg as std_msgs
from ultralytics import YOLO
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics.utils.plotting import Annotator
from collections import defaultdict
from collections import OrderedDict
import cv2.aruco as aruco
from cv2.aruco import Dictionary
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from scipy.interpolate import CubicSpline
from scipy.integrate import quad

cap=cv2.VideoCapture(2)

#red colour
lower_range=np.array([110,55,79])
upper_range=np.array([179,255,255])

#yellow colour
lower_range1=np.array([22,162,119])
upper_range1=np.array([101,255,255])

#blue colour
lower_range2=np.array([6,227,24])
upper_range2=np.array([179,255,255])

class Lidar_Detection():
    def __init__(self):
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.pub = rospy.Publisher('/motion', WheelRpm, queue_size=10)
        rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)
        rospy.Subscriber('/zed2i/zed_node/rgb/image_rect_color',Image, self.color_callback)
        rospy.Subscriber('/zed2i/zed_node/depth/depth_registered', Image, self.depth_callback)

        #initialize realsense camera
        # self.pipeline = rs.pipeline()
        # config = rs.config()
        # config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        # self.pipeline.start(config)

        self.thresh = 0.5
        self.left_check = 10
        self.right_check = 10
        self.midpoint_list = []
        origin = [0,0]
       # self.midpoint_list.append()
        self.odom_self = [0,0]
        self.initial_odom = [0,0]
        self.memory_odom = [0,0]
        self.total_distance = 0
        self.midpoint_array = np.array((0))
        self.counter = 0
        self.distance = 0
        self.odom_initialized = False
        self.timer = time.time()
        self.final_list = []
        self.distance = 0
        self.reached_end = False                      # Will change it to true when the cardboard centre distance is being received.    
        self.distance = 0                   # distance of camera from the cardboard.# Using Realsense/Zed 2i ####
        self.color_came= False
        self.depth_came = False
        self.depth_color = 0.0
        self.color_detected = False
        self.distance_spline = 0
        self.counter_final = 0
        self.x =[]
        self.y = []
        self.memory_updated = False
    def color_callback(self, data):
        try:
            bridge = CvBridge()
            self.color_image = bridge.imgmsg_to_cv2(data, 'passthrough')
            self.color_came = True
        except CvBridgeError as e:
            print(e)

    def depth_callback(self, data):
        try:
            bridge = CvBridge()
            self.depth_image = bridge.imgmsg_to_cv2(data,'passthrough')
            self.depth_came = True
           # print("shape", np.shape(self.depth_image))
        except CvBridgeError as e:
            print(e)


    def lidar_callback(self, data):
        right_num = int(0.35*len(data.ranges))
        left_num = int(0.65*len(data.ranges))
        left_check_sum = 0
        right_check_sum = 0
        print("check1")
        data.ranges = np.nan_to_num(data.ranges, copy = True, nan = 0.8, posinf = 1000, neginf = 1000)
        print(f"Left Points :{data.ranges[left_num:left_num+30]}, Right Points: {data.ranges[right_num:right_num+30]}")
        for i in range(100):
            #if not np.isnan(data.ranges[left_num+i]):
            #print(data.ranges[left_num+i])
            left_check_sum = data.ranges[left_num+i] + left_check_sum
            #if np.isnan(data.ranges[right_num+i]):
            #print(data.ranges[right_num+i])
            right_check_sum = data.ranges[right_num+i] + right_check_sum
        self.left_check = left_check_sum/100
        print("left", self.left_check)
        self.right_check = right_check_sum/100
        print("right", self.right_check)

    def odom_callback(self, data):  #This callback gives the current x and y coordinates. 
        if self.memory_updated == True:  ##should be reduced to approx 6 but more than 6 as it gets executed in line 254
            self.timer  = time.time() 
            self.memory_odom = self.odom_self
            self.memory_updated = False

        self.odom_self[0] = data.pose.pose.position.x
        self.odom_self[1] = -data.pose.pose.position.y
        if self.odom_initialized == False:
            self.initial_odom[0] = self.odom_self[0]
            self.initial_odom[1] = self.odom_self[1]
            self.odom_initialized = True
        self.odom_self[0] = self.odom_self[0] - self.initial_odom[0]
        self.odom_self[1] = self.odom_self[1] - self.initial_odom[1]

    def spline_distance(self,x, y):
        # Fit a cubic spline to the points
        cs_x = CubicSpline(range(len(x)), x)
        cs_y = CubicSpline(range(len(y)), y)
        print('cs_x,cs', x,y)
        
        # Define the integrand for the arc length calculation
        def integrand(t):
            print("came inside integrand########################")
            dx_dt = cs_x(t, 1)  # First derivative of x with respect to t
            dy_dt = cs_y(t, 1)  # First derivative of y with respect to t
            return np.sqrt(dx_dt**2 + dy_dt**2)
        
        # Integrate the arc length from t=0 to t=len(x)-1
        arc_length, _ = quad(integrand, 0, len(x)-1)
        print("type of cs_x is:",arc_length)

        return arc_length
    
    #Detecting colour panel
    def color_detection(self): 
        # Red Colour
        frame = self.color_image
        x1, y1, x2, y2, x3, y3 = 0,0,0,0,0,0
        hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        mask=cv2.inRange(hsv,lower_range,upper_range)
        _,mask1=cv2.threshold(mask,254,255,cv2.THRESH_BINARY)
        cnts,_=cv2.findContours(mask1,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        #a=[]
        print("shape of depth image ", np.shape(self.depth_image))
        print("shape of color image", np.shape(self.color_image))

        for c in cnts:
            x=600
            if cv2.contourArea(c)>x:
                x,y,w,h=cv2.boundingRect(c)
                #a.append(x)
                x1=int(x+x+w)//2
                y1=int(y+y+w)//2
                #cv2.circle(self.color_image,(x1,y1),4,(255,0,255),-1)
                #cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255),2)
                #cv2.putText(frame,("DETECT"),(10,60),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,0,255),2)

        # Blue COlour
        hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        mask=cv2.inRange(hsv,lower_range2,upper_range2)
        _,mask1=cv2.threshold(mask,254,255,cv2.THRESH_BINARY)
        cnts,_=cv2.findContours(mask1,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        #c=[]
        for c in cnts:
            x=600
            if cv2.contourArea(c)>x:
                x,y,w,h=cv2.boundingRect(c)
                #c.append(x)
                x3=int(x+x+w)//2
                y3=int(y+y+w)//2
                #cv2.circle(self.color_image,(x3,y3),4,(255,0,255),-1)
                #cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0 ),2)
                #cv2.putText(frame,("DETECT"),(10,60),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,0,255),2)

        # Yellow Colour
        hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        mask=cv2.inRange(hsv,lower_range1,upper_range1)
        _,mask1=cv2.threshold(mask,254,255,cv2.THRESH_BINARY)
        cnts,_=cv2.findContours(mask1,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        #b=[]
        for c in cnts:
            x=600
            if cv2.contourArea(c)>x:
                x,y,w,h=cv2.boundingRect(c)
                #b.append(x)
                x2=int(x+x+w)//2
                y2=int(y+y+w)//2
                #cv2.circle(self.color_image,(x2,y2),4,(255,0,255),-1)
                #cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
                #cv2.putText(frame,("DETECT"),(10,60),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,0,255),2)

        self.color_detected = True
        if x1 != 0 and y1 != 0 and x2 ==0 and x3 ==0 and y2 == 0 and y3 ==0:
            self.depth_color = self.depth_image[y1,x1]
            print("red detected")
        elif x1 == 0 and y1 == 0 and x2 !=0 and x3 ==0 and y2!= 0 and y3 ==0:
           self.depth_color=self.depth_image[y2,x2]
           print("yellow detected")
        elif x1 == 0 and y1 == 0 and x2 ==0 and x3 !=0 and y2== 0 and y3 !=0:
           self.depth_color=self.depth_image[y3,x3]
           print("blue detected")
        else:
            self.color_detected= False
            print("Nothing detected")
        cv2.imshow("FRAME",frame)
        if cv2.waitKey(1)&0xFF==27:
            cv2.destroyAllWindows()


    def main(self):
        twist = WheelRpm()                         ## moving rover ahead through the tunnel while assuring that it doesnt hit the walls.
        twist.vel = 20
        safety_distance=0.8
        if self.left_check <= safety_distance:
            twist.omega = -20
        if self.right_check<=safety_distance:
            twist.omega = 20
        self.pub.publish(twist)                ##
        print("self.odom", self.odom_self)
        

                                              ### storing the values of postion(x,y) of rover in self.midpoint_list till a threshold value.
        self.counter +=1                      
        midpoint_tuple = self.odom_self 
        print("midpoint tuple", midpoint_tuple)
        self.midpoint_list.append(midpoint_tuple)
        print("I came inside main")

        if((time.time() - self.timer)>2):
        #if self.memory_updated == True:
            self.timer  = time.time() 
            self.distance+= np.sqrt((self.odom_self[1]-self.memory_odom[1])**2 + (self.odom_self[0]-self.memory_odom[0])**2)
            print("distance w/o --> normal ", self.distance)
            self.memory_updated = True

                                              ###

       # threshold = 20                         ## Will set it acc. to the testing.
        if self.color_came== True and self.depth_came == True:
            self.color_detection()
            
        self.x.append(midpoint_tuple[0])
        self.y.append(midpoint_tuple[1])

        if  self.counter == 50:
            x = list(self.x)
            y = list(self.y)

            points = list(zip(x,y))
            # Sort the list of tuples based on the first element of each tuple
            sorted_points = sorted(points, key=lambda x: x[0])
            x, y = zip(*sorted_points)
            x = list(x)
            y = list(y)

            distance = self.spline_distance(x, y)
            print("########### dist ################## ",distance)
            self.distance_spline = distance 
            self.counter = 0
            print(f"The distance along the cubic spline is: {self.distance_spline:.2f}")
        
        if self.color_detected == True and self.counter_final == 0 :
            x = list(self.x)
            y = list(self.y)

            points = list(zip(x,y))
            # Sort the list of tuples based on the first element of each tuple
            sorted_points = sorted(points, key=lambda x: x[0])
            x, y = zip(*sorted_points)
            x = list(x)
            y = list(y)

            distance = self.spline_distance(x, y)
            print("########### dist ################## ",distance)
            self.distance_spline = distance
            self.counter = 0
            print(f"The distance of colored object is: {self.depth_color:.2f}")
            self.counter_final += 1
            self.distance = self.distance + self.depth_color
            print(f"The Final distance is ---->   {self.distance:.2f}")



    def spin(self):
        while not rospy.is_shutdown():
            self.main()
            rate.sleep()

        else:
            print("hi")

if __name__=="__main__":
    rospy.init_node("lava", anonymous=True)
    rate = rospy.Rate(10)
    ahh = Lidar_Detection()
    ahh.spin()
