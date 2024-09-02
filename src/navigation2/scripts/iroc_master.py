#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, Int32
from traversal.msg import WheelRpm
import math
import time
import cmath
from navigation2.msg import red

class Master():
    def __init__(self):
        rospy.Subscriber('/pranav_red', red, self.pranav_callback)
        rospy.Subscriber('/tanish_red', red, self.tanish_callback)
        rospy.Subscriber('/anuj_red', red, self.anuj_callback)
        rospy.Subscriber('/goal_ik',Int32,self.ik_red_callback)
        rospy.Subscriber('/goal_blue',Int32,self.ik_blue_callback) 
        self.pranav_vel = 0
        self.pranav_omega = 0
        self.pranav_detect = 0
        self.tanish_vel = 0
        self.tanish_omega = 0
        self.tanish_detect = 0
        self.anuj_vel = 0
        self.anuj_omega = 0
        self.anuj_detect = 0
        self.tanish_counter = 0
        self.anuj_counter = 0
        self.rate = rospy.Rate(10)
        self.vel_pub = rospy.Publisher('/motion', WheelRpm, queue_size = 10)
        self.ik_start_tanish=0
        self.ik_start_anuj = 0
        self.ik_over = False
        self.listen_bool = False #to decide to listen to Tanish or Anuj

    def pranav_callback(self, msg):
        self.pranav_vel = msg.vel
        self.pranav_omega = msg.omega
        self.pranav_detect = msg.detect
    
    def tanish_callback(self, msg):
        self.tanish_vel = msg.vel
        self.tanish_omega = msg.omega
        self.tanish_detect = msg.detect
        #self.counter = msg.counter

    def anuj_callback(self, msg):
        self.anuj_vel = msg.vel
        self.anuj_omega = msg.omega
        self.anuj_detect = msg.detect
        #self.anuj_counter = msg.counter
    
    def ik_red_callback(self,msg):
        if(msg.data == 0 and self.ik_start_tanish == 1):
            self.listen_bool = not self.listen_bool
        self.ik_start_tanish = msg.data
    
    def ik_blue_callback(self,msg):
        if(msg.data == 0 and self.ik_start_anuj == -1):
            self.listen_bool = not self.listen_bool
        self.ik_start_anuj=msg.data

    def main(self):
        final_ctrl = WheelRpm()
        if(self.ik_start_tanish == 1 or self.ik_start_anuj== -1):
            print(f"IK Has started. Rover stopped. self.ik_start_tanish = {self.ik_start_tanish}, self.ik_start_anuj = {self.ik_start_anuj}")
            final_ctrl.vel = 0
            final_ctrl.omega = 0
        elif(self.listen_bool == False): #listen to Tanish
            print("Listening To Tanish")            
            if(self.tanish_detect == True) and (self.pranav_detect == False or self.tanish_vel == 0):
                final_ctrl.vel = self.tanish_vel
                final_ctrl.omega = self.tanish_omega
                print("Red Colour Detection. Tanish Giving values")
            elif (self.pranav_detect == True):
                final_ctrl.vel = self.pranav_vel
                final_ctrl.omega = self.pranav_omega
                print("Pranav Obstacle Avoidance")
            else:
                final_ctrl.vel = self.pranav_vel
                final_ctrl.omega = self.pranav_omega
                print("Pranav Go To Goal Values")
                
        elif (self.listen_bool == True): #listen to anuj
            print("Listening to Anuj")
            if (self.anuj_detect == True) and (self.pranav_detect == False):
                final_ctrl.vel = self.anuj_vel
                final_ctrl.omega = self.anuj_omega
                print("Anuj Blue Colour detection. Anuj giving values")
            elif (self.pranav_detect == True):
                final_ctrl.vel = self.pranav_vel
                final_ctrl.omega = self.pranav_omega
                print("Pranav Obstacle Avoidance")
            else:
                final_ctrl.vel = self.pranav_vel
                final_ctrl.omega = self.pranav_omega
                print("Pranav Go To Goal Values")

        print(final_ctrl)
        self.vel_pub.publish(final_ctrl)
    def spin(self):
        while not rospy.is_shutdown():
            self.main()
            self.rate.sleep()

if __name__ =="__main__":
    rospy.init_node("master")
    ahhh = Master()
    ahhh.spin()


