#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import imutils 
import numpy as np
from sensor_msgs.msg import Imu, NavSatFix
from traversal.msg import WheelRpm
from navigation2.msg import auto
from std_msgs.msg import Int32, Int8

class GoToGoal():
    def __init__(self):
        #rospy.init_node('go_to_goal', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/motion', WheelRpm, queue_size=10)
        rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)
        rospy.Subscriber('/imu', Imu, self.imu_callback)
        rospy.Subscriber('/gps_coordinates', NavSatFix, self.gps_callback)
        rospy.Subscriber('/rscp_data', auto, self.rscp_callback)
        self.check_publisher=rospy.Publisher('/check',Int32,queue_size=100)
        self.task_completed_pub = rospy.Publisher('/task_completed', Int32, queue_size=100)
        self.led_pub=rospy.Publisher('/rover_state',Int32,queue_size=10)
        self.rate = rospy.Rate(10)
        self.dummy = 0
        self.odom_self = [0,0]
        self.initial_odom = [0,0]
        self.a = 0
        self.initial_yaw = 0
        self.current_pitch,self.current_roll, self.current_yaw=0,0,0
        self.current_latitude, self.current_longitude = 0,0
        self.angle_to_turn = 0
        self.distance_calculated = False
        self.angle_calculated = False
        self.odom_initialized= False
        #self.gps_publisher = rospy.Publisher("gps_from_location1", gps_data, queue_size=10)
        self.lat_list = []
        self.long_list = []
        self.gps_received = False
#from gps data include coordinates here
        self.goal_lat = 0 #default
        self.goal_long = 0 #default
        self.rscp_received = False
        self.execute_goal=True
        self.check_msg=Int32()
        self.check_msg.data=0
        self.second_gps_received = False
        self.initial_lat = 0
        self.initial_long = 0
        self.counter = 0 #publish the check value only once after completion
        self.rover_state=Int32()
        self.code_stopping_variable=False
        self.rover_state=0 #initially its in red colour
        self.gps_counter=0
        self.counter_gps_two=0
        self.gps_two_stop=0

    def rscp_callback(self,data):
        print(self.gps_counter)
        if(data.msg_id == 2 and self.gps_counter==0):
            print("Im receiving coordinates from rscp")
            self.rscp_received = True
            self.goal_lat = data.latitude
            self.goal_long = data.longitude
            self.rover_state=1
            self.led_pub.publish(self.rover_state)
            self.gps_counter=self.gps_counter+1
            
        elif data.msg_id == 10:
            self.initial_lat = data.latitude
            self.initial_long = data.longitude
            self.rover_state=1
            self.led_pub.publish(self.rover_state)
            print(f"In Callback: {self.initial_lat}, {self.initial_long}")

        elif self.gps_counter == 1 and data.msg_id == 2:
            print("stageId = 2")
            self.second_gps_received = True
            self.initial_lat = self.goal_lat
            self.initial_long = self.goal_long
            self.goal_lat = data.latitude
            self.goal_long = data.longitude
            self.counter_gps_two==1
    
    def distance_to_goal(self):
        a = math.pow(math.sin((self.goal_lat - self.initial_lat)*math.pi/360),2) + math.cos(self.goal_lat*math.pi/180)*math.cos(self.initial_lat*math.pi/180)*math.pow(math.sin((self.goal_long - self.initial_long)*math.pi/360),2)
        c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))
        self.distance = 6371 * c *1000
        self.distance_calculated = True
        
    def odom_callback(self, data):
        #print("Odometry callback called.")
        #self.current_latitude = msg.pose.pose.position.x
        #self.current_longitude = msg.pose.pose.position.y
        #self.current_pose = msg.pose.pose
        #rospy.loginfo("Current Pose: {}".format(self.current_pose))
        self.odom_self[0] = data.pose.pose.position.x
        self.odom_self[1] = -data.pose.pose.position.y
        if self.odom_initialized == False:
            self.initial_odom[0] = self.odom_self[0]
            self.initial_odom[1] = self.odom_self[1]
            self.odom_initialized = True
        self.odom_self[0] = self.odom_self[0] - self.initial_odom[0]
        self.odom_self[1] = self.odom_self[1] - self.initial_odom[1]
    
    def imu_callback(self, data):
        current_x = data.orientation.x
        current_y = data.orientation.y
        current_z = data.orientation.z
        current_w = data.orientation.w
        current_tuple=(current_x, current_y, current_z, current_w)
        self.current_pitch, self.current_roll, self.current_yaw = euler_from_quaternion(current_tuple)
        if self.initial_yaw == 0:
            self.initial_yaw = self.current_yaw*180/math.pi
        self.current_yaw = self.current_yaw*180/math.pi - self.initial_yaw
#        print("self.current_yaw", self.current_yaw)
        
        if self.current_yaw < -179:
            self.current_yaw = self.current_yaw + 360
        elif self.current_yaw > 179:
            self.current_yaw = self.current_yaw - 360


    def get_angle_to_goal(self):
        # if hasattr(self, 'current_pose'):  # Check if current_pose attribute exists
        a = math.atan2(self.goal_long - self.initial_long, self.goal_lat - self.initial_lat)
        self.angle_to_turn = self.current_yaw - a*180/math.pi
        print(self.angle_to_turn)
        if self.angle_to_turn <= 0:
            rotation = "left"
            self.dummy = -1
        elif self.angle_to_turn > 0:
            rotation = "right"
            self.dummy = 1
        print("rotating", rotation)
        self.angle_calculated = True
        #else:
         #   rospy.logwarn("No 'current_pose' attribute. Skipping angle calculation.")
    def move_to_goal(self):
        print("distance to cover",self.distance)
        print(f"Initial Odometry: {self.initial_odom}")
        distance_travelled = np.sqrt((self.odom_self[1]-self.initial_odom[1])**2 + (self.odom_self[0]-self.initial_odom[0])**2)
        print("distance travelled", distance_travelled)
        vel_msg = WheelRpm()
        if abs(distance_travelled -self.distance) < 0.5:
            rospy.loginfo("Goal reached!")
            vel_msg.vel = 0
            vel_msg.omega = 0
            self.velocity_publisher.publish(vel_msg)
            self.rover_state=1
            self.led_pub.publish(self.rover_state)
            self.code_stopping_variable=True
        else:
            print("inside move_to_goal",self.current_yaw)
            self.rover_state=2
            self.led_pub.publish(self.rover_state)
            print("abs(self.current_yaw - self.angle_to_turn)", abs(self.current_yaw - self.angle_to_turn), "self.second_gps_received", self.second_gps_received)            
            if (self.current_yaw - self.angle_to_turn) < -4:
                #vel_msg = WheelRpm()
                vel_msg.omega = 20 
                vel_msg.vel = 0
                # self.velocity_publisher.publish(vel_msg)
            elif(self.current_yaw - self.angle_to_turn) > 4:
                #vel_msg = WheelRpm()
                vel_msg.omega = 20* (-1)
                vel_msg.vel= 0
                # self.velocity_publisher.publish(vel_msg)
            elif abs(self.current_yaw - self.angle_to_turn) < 4:
                #vel_msg = WheelRpm()
                vel_msg.vel = 20
                vel_msg.omega= 0
 
            else:
                vel_msg.vel = 0
                vel_msg.omega = 0
                if self.counter_gps_two == 1:
                    self.gps_two_stop=1
                # self.velocity_publisher.publish(vel_msg)
        self.velocity_publisher.publish(vel_msg)
        self.rate.sleep()

        if self.gps_two_stop==1:
                print("code stopping here")
                self.check_msg.data=1
                self.execute_goal=False #Goal has reached
                self.code_stopping_variable = True
 



    def gps_callback(self,msg):
        counter = 0
        if(msg.latitude and  msg.longitude) and self.a == 0:
            counter = counter +1            
            self.current_latitude=msg.latitude
            self.current_longitude=msg.longitude
            self.lat_list.append(self.current_latitude)
            self.long_list.append(self.current_longitude)
            print("counter", counter)
            if counter >= 100:
                self.current_latitude = sum(self.lat_list)/len(self.lat_list)
                self.current_longitude = sum(self.long_list)/len(self.long_list)
                self.gps_received = True
                self.a += 1
        
    def spin(self):
        while not rospy.is_shutdown():
            print(self.code_stopping_variable)
            if (self.code_stopping_variable==False): # check if execute_goal is True
                print("check1 in spin", self.code_stopping_variable)
                self.main()
                self.check_publisher.publish(self.check_msg) #setting initial value of counter as 0 so that gps code runs
            elif self.counter == 0:
                print("check2 in main")
                self.check_publisher.publish(self.check_msg) #setting initiali value of counter as 0 so that gps code runs
                self.task_completed_pub.publish(1)
                self.code_stopping_variable=False
                self.counter = self.counter +1
                self.distance_calculated = False
                self.angle_calculated = False
            elif self.execute_goal==False and self.counter>=1:
                print("check3 in spin")
                self.check_publisher.publish(self.check_msg) 
                self.counter +=1
            self.rate.sleep()

    def main(self):
        if (self.rscp_received == True) or self.second_gps_received == True:
            if self.distance_calculated == False:
                self.distance_to_goal()
            if self.angle_calculated == False:
                self.get_angle_to_goal()
            self.move_to_goal()
        print("Where am i?", self.initial_lat, self.initial_long)
        print("goal", self.goal_lat, self.goal_long)
        print("angle_to_turn", self.angle_to_turn)
        print("self.current_yaw", self.current_yaw)


if __name__ == '__main__':
    try:
        rospy.init_node('go_to_goal', anonymous=True)
        auto = GoToGoal()
        auto.spin()
    except rospy.ROSInterruptException:
        pass

