#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
import cv2
import math
import imutils
import torch
import torchvision.transforms as transforms
from threading import Lock, Thread
import numpy as np
import queue
from traversal.msg import WheelRpm
import math
import time

PI=math.pi

class TanBug:
    flag=-1
    def __init__(self,yaw_angle):
        # Initialize the node
        rospy.init_node('tangent_bug_rover', anonymous=True)

        # Define the goal location (x, y)
        self.xgoal=5.0
        self.ygoal=0.1
        self.goal = [self.xgoal,self.ygoal]  # Change this as needed

        # Create a publisher for the cmd_vel topic to control the rover
        self.cmd_vel = rospy.Publisher('/motion', WheelRpm, queue_size = 10)

        # Create subscribers for odometry and laser scan data
        rospy.Subscriber('/zed2i/zed_node/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)



        # Rover state
        self.pose = None
        self.yaw = 0
        self.scan_data = None
        self.safe_distance = 1.5 # Distance to maintain from obstacles
        self.speed = 1
        self.angular_speed = 1
        self.following_obstacle = False
        self.follow_boundary_direction = None  # Left (-1) or Right (1) boundary following
        self.right_range=0
        self.left_range=0
        self.center_range=0

        self.yaw_error=10
        self.dir=0
        self.angle_to_goal=15

        self.temp_yaw=0
        self.front_detect_angle = 40 # 20 degrees
        
        self.twist=WheelRpm()
    

    def odom_callback(self, msg):
        # Get current position and orientation of the robot
        self.pose = msg.pose.pose
        orientation_q = self.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.yaw = yaw

        self.angle_to_goal = self.get_angle_to_goal()
        self.yaw_error = int(self.angle_to_goal - self.yaw)

        if TanBug.flag==-1:
            self.x_ini=self.pose.position.x
            self.y_ini=self.pose.position.y
            
            self.yaw_ini=self.yaw
            TanBug.flag=0
        else:
            self.pose.position.x-=self.x_ini
            self.pose.position.y-=self.y_ini
            self.yaw -= self.yaw_ini 
            
        print(f"Current x is {self.pose.position.x} and current y is {self.pose.position.y}")
        print(f"goal x is {self.goal[0]} and goal y is {self.goal[1]}. Distance from goal is {self.get_distance_to_goal()}")

    def converter(self,s):
        if s==float('inf'):
            return 100
        else:
            return s

    def laser_callback(self, msg):
        # Get the laser scan data
        self.scan_data = np.array(msg.ranges)
        self.scan_data=np.array(list(map(self.converter,list(self.scan_data))))
        self.right_range = np.mean(self.scan_data[len(self.scan_data)//4 :len(self.scan_data)*5 // 12])  # Left side
        self.center_range = np.mean(self.scan_data[len(self.scan_data)*5//12 : len(self.scan_data)*7 // 12])  # Right side
        self.left_range=np.mean(self.scan_data[len(self.scan_data) *7 // 12: len(self.scan_data) * 3 // 4])

    def get_distance_to_goal(self):
        if self.pose is None:
            return float('inf')
        position = self.pose.position
        goal_distance = math.sqrt((self.goal[0] - position.x) ** 2 + (self.goal[1] - position.y) ** 2)
        print(f"Goal DIstance = {goal_distance}")
        return goal_distance

    def get_angle_to_goal(self):
        if self.pose is None:
            return 0
        position = self.pose.position
        delta_x = self.goal[0] - position.x
        delta_y = self.goal[1] - position.y
        angle_to_goal = math.atan2(delta_y, delta_x)
        print(f"Angle to Goal = {angle_to_goal}")
        return angle_to_goal

    def is_obstacle_in_front(self):
        if self.scan_data is None:
            return False
        # Consider the middle 10 degrees of laser scan readings
        self.front_indices = range(len(self.scan_data) // 2 - self.front_detect_angle, len(self.scan_data) // 2 + self.front_detect_angle) #20 degrees either side
        return np.any(self.scan_data[self.front_indices] < self.safe_distance)

    def get_obstacle_direction(self):
        if self.left_range < self.right_range:
            return 1  # Turn right
        else:
            return -1  # Turn left

    def turn_to_goal(self):
        if abs(self.yaw_error)>0.01:
            if self.angle_to_goal>self.yaw:
                self.twist.vel=0
                self.twist.omega = 40
            else:
                self.twist.omega = -40
                self.twist.vel=0
        else:
            self.twist.omega=0
            TanBug.flag=1
        self.cmd_vel.publish(self.twist)

    def goStraight(self):
        if self.is_obstacle_in_front():
            self.twist.vel=0
            self.dir=self.get_obstacle_direction()
            TanBug.flag=2
        else:
            self.twist.vel=25
            self.twist.omega=int(self.yaw_error)
        self.cmd_vel.publish(self.twist)

    def steer(self):
        if self.is_obstacle_in_front():
            self.twist.omega = self.angular_speed * self.dir *2
            self.twist.vel = int(self.speed * np.mean(self.scan_data[self.front_indices])) # Slow down while following boundary

            #print(self.twist.vel)

        else:
            start=time.time()
            while time.time()-start <3:
                self.twist.vel=0
                self.twist.omega=15*self.dir
                self.cmd_vel.publish(self.twist)
            self.twist.vel=0
            self.twist.omega=0
            TanBug.flag=int(-0.5*(self.dir)+3.5)
            self.dir=0

        self.cmd_vel.publish(self.twist)



    def follow_boundary_left(self):
        #print((self.left_range)/2)
        # print("Left data is",self.left_range)
        # print(self.scan_data[-len(self.scan_data)//3:])

        if self.left_range<5.0:
            if (self.center_range+self.left_range)/2 <10:
                self.dir=self.get_obstacle_direction()
                TanBug.flag=2
            else:
                self.twist.vel=15
                self.twist.omega=-20
                self.cmd_vel.publish(self.twist)
                print("Left data is",self.left_range)
        else:
            start=time.time()
            while time.time()-start <5:
                self.twist.vel=15
                self.twist.omega=0
                self.cmd_vel.publish(self.twist)
            self.temp_yaw=self.yaw
            TanBug.flag=5

    def follow_boundary_right(self):
        #print((self.right_range)/2)
        # print("Left data is",self.left_range)
        # print(self.scan_data[:len(self.scan_data)//3])

        if self.right_range<5.0:
            if (self.center_range+self.right_range)/2 <10:
                self.dir=self.get_obstacle_direction()
                TanBug.flag=2
            else:
                self.twist.vel=15
                self.twist.omega=10
                self.cmd_vel.publish(self.twist)
                print("Right val is",self.right_range)
        else:
            start=time.time()
            while time.time()-start <5:
                self.twist.vel=15
                self.twist.omega=0
                self.cmd_vel.publish(self.twist)
            self.temp_yaw=self.yaw
            TanBug.flag=5

    def arc(self,yaw0,sign):
        yaw_cur=self.yaw

        if yaw_cur*yaw0<0:
            yaw_cur+=PI*sign

        #print(yaw0)
        #print(yaw_cur)
        if abs(yaw_cur-yaw0)<PI/2:
            if self.is_obstacle_in_front():
                self.twist.vel=0
                self.twist.omega=0
                TanBug.flag=2

            else:
                self.twist.vel=15
                self.twist.omega=int(-10*sign)

            self.cmd_vel.publish(self.twist)
        else:
            start=time.time()
            while time.time()-start <3:
                self.twist.vel=15
                self.twist.omega=0
                self.cmd_vel.publish(self.twist)
            TanBug.flag=0




    def main(self):
        rate = rospy.Rate(10)

        while self.get_distance_to_goal()>0.3:
            print(f"Tanbug flag is {TanBug.flag}")
            print(f"Yaw error is {self.yaw_error}")
            
            if TanBug.flag==0:
                self.turn_to_goal()
            elif TanBug.flag==1:
                self.goStraight()
            elif TanBug.flag==2:
                self.steer()
            elif TanBug.flag==3:
                self.follow_boundary_right()
            elif TanBug.flag==4:
                self.follow_boundary_left()
            elif TanBug.flag==5:
                self.arc(self.temp_yaw,(self.temp_yaw/abs(self.temp_yaw)))

            rate.sleep()

        self.twist.vel=0
        self.twist.omega=0
        self.cmd_vel.publish(self.twist)

        print("Goal Reached! Error:",self.get_distance_to_goal())

if __name__ == '__main__':
    try:
        # Initialize the Tangent Bug Algorithm on the rover
        yaw_angle=5
        rover = TanBug(yaw_angle)
        rover.main()
        # rover.spin()
    except rospy.ROSInterruptException:
        pass

