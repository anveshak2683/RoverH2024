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

class GoToGoal():
    def __init__(self):
        self.velocity_publisher = rospy.Publisher('/motion', WheelRpm, queue_size=10)
        rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)
        rospy.Subscriber('/imu', Imu, self.imu_callback)
        rospy.Subscriber('/gps_coordinates', NavSatFix, self.gps_callback)
        rospy.Subscriber('/rscp_data', auto, self.rscp_callback)
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
        self.second_gps_received = False

    def rscp_callback(self,data):
        if(data.msg_id == 2):
            self.rscp_received = True
            self.goal_lat = data.latitude
            self.goal_long = data.longitude
        if data.msg_id == 10:
            self.initial_lat = data.latitude
            self.initial_long = data.longitude
        if data.stage_id == 4:
            self.second_gps_received = True
            self.initial_lat = self.goal_lat
            self.initial_long = self.goal_long
            self.goal_lat = data.latitude
            self.goal_long = data.longitude 
    
    def distance_to_goal(self):
        a = math.pow(math.sin((self.goal_lat - self.initial_lat)*math.pi/360),2) + math.cos(self.goal_lat*math.pi/180)*math.cos(self.initial_lat*math.pi/180)*math.pow(math.sin((self.goal_long - self.initial_long)*math.pi/360),2)
        c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))
        print(f"a = {a}, c = {c}")
        self.distance = 6371 * c *1000
        self.distance_calculated = True
        #self.distance = math.sqrt((self.goal_lat - self.current_latitude)**2 +(self.goal_long - self.current_longitude)**2)
        
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
        if self.angle_to_turn < 0:
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
            gps_msg = gps_data()
            gps_msg.latitude = self.current_latitude
            gps_msg.longitude = self.current_longitude
            #self.gps_publisher.publish(gps_msg)
             # haversine used to get the distance
        else:
            print("inside move_to_goal",self.current_yaw)
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
            else:
                #vel_msg = WheelRpm()
                vel_msg.vel = 20
                vel_msg.omega= 0
                # self.velocity_publisher.publish(vel_msg)
        self.velocity_publisher.publish(vel_msg)
        self.rate.sleep()

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
            self.main()
            self.rate.sleep()

    def main(self):
        if (self.gps_received == False and self.rscp_received == True) or self.second_gps_received == True:
            if self.distance_calculated == False:
                self.distance_to_goal()
            if self.angle_calculated == False:
                self.get_angle_to_goal()
            self.move_to_goal()
        print("Where am i?", self.odom_self)
        print("goal", self.goal_lat, self.goal_long)
        print("angle_to_turn", self.angle_to_turn)
        print("self.current_yaw", self.current_yaw)


if __name__ == '__main__':
    try:
        rospy.init_node('go_to_goal')
        rate = rospy.Rate(10)
        auto = GoToGoal()
        auto.spin()
    except rospy.ROSInterruptException:
        pass

