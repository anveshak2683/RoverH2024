#!/usr/bin/env python

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

class TangentBugRover:
    def __init__(self,yaw_angle):
        # Initialize the node
        rospy.init_node('tangent_bug_rover', anonymous=True)
        
        # Define the goal location (x, y)
        x=float(input("Enter X Coordinate: "))
        y=float(input("Enter Y Coordinate: "))
        self.goal = [x,y]  # Change this as needed
        
        # Create a publisher for the cmd_vel topic to control the rover
        self.cmd_vel_pub = rospy.Publisher('/galileo/cmd_vel', Twist, queue_size=10)

        # Create subscribers for odometry and laser scan data
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/galileo/laser_scan', LaserScan, self.laser_callback)

        # Rover state
        self.pose = None
        self.yaw = 0.0
        self.scan_data = None
        self.safe_distance = 1.5 # Distance to maintain from obstacles
        self.speed = 1
        self.angular_speed = 1
        self.following_obstacle = False
        self.follow_boundary_direction = None  # Left (-1) or Right (1) boundary following
        self.right_range=[]
        self.left_range=[]

    def odom_callback(self, msg):
        # Get current position and orientation of the robot
        self.pose = msg.pose.pose
        orientation_q = self.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.yaw = yaw
        
        self.front_indices=None


    def converter(self,s):
        if s==float('inf'):
            return 1000
        else:
            return s

    def laser_callback(self, msg):
        # Get the laser scan data
        self.scan_data = np.array(msg.ranges)
        self.scan_data=np.array(list(map(self.converter,list(self.scan_data))))
        self.left_range = np.mean(self.scan_data[:len(self.scan_data) // 3])  # Left side
        self.right_range = np.mean(self.scan_data[-len(self.scan_data) // 3:])  # Right side
        self.front_indices = range(len(self.scan_data) // 2 - 10, len(self.scan_data) // 2 + 10)

        print("left range is ",self.left_range)
        print("right range is ",self.left_range)
        print("front range is", np.mean(self.scan_data[self.front_indices]))

    def get_distance_to_goal(self):
        """Calculates the Euclidean distance from the current position to the goal."""
        if self.pose is None:
            return float('inf')
        position = self.pose.position
        return math.sqrt((self.goal[0] - position.x) ** 2 + (self.goal[1] - position.y) ** 2)

    def get_angle_to_goal(self):
        """Calculates the angle between the rover's heading and the goal."""
        if self.pose is None:
            return 0
        position = self.pose.position
        delta_x = self.goal[0] - position.x
        delta_y = self.goal[1] - position.y
        angle_to_goal = math.atan2(delta_y, delta_x)
        return angle_to_goal

    def is_obstacle_in_front(self):
        """Checks if there's an obstacle directly in front of the robot."""
        if self.scan_data is None:
            return False
        # Consider the middle 10 degrees of laser scan readings
        
        return np.any(self.scan_data[self.front_indices] < self.safe_distance)

    def get_obstacle_direction(self):
        """Determines whether to turn left or right to avoid an obstacle."""
        if self.left_range < self.right_range:
            return 1  # Turn right
        else:
            return -1  # Turn left

    def follow_obstacle_boundary(self):
        """Handles obstacle avoidance by following along the obstacle's boundary."""
        twist = Twist()
        if self.follow_boundary_direction is None:
            # Decide to follow left or right boundary
            self.follow_boundary_direction = self.get_obstacle_direction()

        # Adjust rover's speed and turn direction based on boundary following
        twist.angular.z = self.angular_speed * self.follow_boundary_direction *2
        twist.linear.x = self.speed * 0.03 * np.mean(self.scan_data[self.front_indices])  # Slow down while following boundary
        self.cmd_vel_pub.publish(twist)
        

    def go_to_goal(self):
        """Navigates towards the goal while avoiding obstacles using the Tangent Bug Algorithm."""
        rate = rospy.Rate(10)  # 10 Hz
        twist = Twist()
        obs_was_front=False
        dir=0

        while not rospy.is_shutdown():
            if self.pose is None or self.scan_data is None or type(self.right_range)==None or type(self.left_range)==None or type(self.scan_data)==None:
                continue

            # Calculate the distance and angle to the goal
            distance_to_goal = self.get_distance_to_goal()
            angle_to_goal = self.get_angle_to_goal()

            # Check if we reached the goal
            if distance_to_goal < 0.2:
                rospy.loginfo("Goal reached!")
                twist.linear.x=0
                twist.angular.z=0
                break

            # Calculate the angular difference between current yaw and goal direction
            yaw_error = angle_to_goal - self.yaw
            yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))  # Normalize between [-pi, pi]
            print("yaw error is: ",yaw_error)
            # print(min(self.scan_data))
            # Check if there's an obstacle in front
            if self.is_obstacle_in_front():
                rospy.loginfo("Obstacle detected, following the boundary.")
                dir=self.get_obstacle_direction()
                self.follow_obstacle_boundary()
                obs_was_front=True
            
            elif obs_was_front:
                if dir==1:
                    self.left_range = np.mean(self.scan_data[:len(self.scan_data) // 3])
                    while self.left_range<3.0:
                        twist.linear.x=1.0
                        self.cmd_vel_pub.publish(twist)
                        self.left_range = np.mean(self.scan_data[:len(self.scan_data) // 3])
                        # print("Left data is",self.left_range)
                else:
                    self.right_range = np.mean(self.scan_data[-len(self.scan_data) // 3:]) 
                    while self.right_range<3.0:
                        twist.linear.x=1.0
                        self.cmd_vel_pub.publish(twist)
                        self.right_range = np.mean(self.scan_data[-len(self.scan_data) // 3:]) 
                        # print("Right data is",self.right_range)
                obs_was_front=False
                
            else:
                # print(yaw_error)
                # Clear the obstacle-following mode if no obstacles are detected
                self.following_obstacle = False
                self.follow_boundary_direction = None

                # Move towards the goal if no obstacles are in the way
                # rospy.loginfo(yaw_error)
                if abs(yaw_error)>0.1:
                    # print("meow")
                    # print(yaw_error)
                    if angle_to_goal>self.yaw:
                        twist.linear.x=0
                        twist.angular.z = 1.0
                    else:
                        twist.angular.z = -1.0
                        twist.linear.x=0
                    self.cmd_vel_pub.publish(twist)

                else:                    
                    twist.linear.x = self.speed
                    twist.angular.z = yaw_error
                    self.cmd_vel_pub.publish(twist)

            rate.sleep()

if __name__ == '__main__':
    try:
        # Initialize the Tangent Bug Algorithm on the rover
        yaw_angle=5
        rover = TangentBugRover(yaw_angle)
        rover.go_to_goal()
    except rospy.ROSInterruptException:
        pass
