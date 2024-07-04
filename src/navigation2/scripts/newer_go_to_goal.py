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
import queue
from traversal.msg import WheelRpm
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
import math
import time

class GoToGoal():
    def __init__(self):
        self.x_pos = 5
        self.y_pos = 0
        self.vel_publisher = rospy.Publisher('/motion', WheelRpm, queue_size = 10)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)

        self.rate = rospy.Rate(10)
        self.current_pose_x = 0
        self.current_pose_y = 0
        self.yaw_angle = 0
        self.imgmsg = Image()
        self.bridge = CvBridge()
        self.torchvision_img = 0
        self.qsize = 5 #size of moving average filter queue
        self.count = 0 #number of iterations the rover stops
        self.tensor = torch.tensor([1]).to(device) #initialize
        self.tensor_prime = torch.tensor([1])
        self.lidar_queue = queue.Queue(self.qsize)
        self.angle_start = 0
        self.first_stop = False
        self.startx = 0
        self.starty = 0
        self.turn_started = False
        self.turn_over = False
        self.second_stop = False
        self.third_stop = False
        self.object_in_front = False #will repeat if number of points less than 2 m during the avoidance is too high. 
        self.exit_signal = False
        self.target_dist = 0
        self.lock = Lock()
        self.num_scans = 0
        self.angle_max = 0 #for lidar
        self.angle_min = 0 #for lidar
        self.left_45 = 0 #for lidar
        self.left_90 = 0
        self.right_90 = 0
        self.right_45 = 0 #for lidar
        self.middle_0 = 0
        self.initial_less_than_2m_points = 0 #to keep rest of the code as a function of the original number of points less than 2m detects
        self.turn_speed = 15
        self.move_speed = 30

        self.time_turn = 0 #to keep a track of turn timing
        self.turn_timing_count_start = False
        self.last_time_turn = 0
        self.displacement_y = 0 #to basially estimate odom with time

        #self.capture_thread = Thread(target=self.moving_average_filter, kwargs=None)
        #self.capture_thread.start()
        self.initial_odom_set = False
        self.initial_angle_set = False

        self.initial_yaw = 0
        self.initial_pose_x = 0
        self.initial_pose_y = 0
         
        ##SPL VARIABLE##
        self.y_disp_too_less = False
        #below code is for orienting rover towards object
        self.aligned_center = False
        self.lr_points = 10
        #rospy.Subscriber('/intel/depth/image_raw', Image, self.depth_callback)
        rospy.Subscriber('/imu', Imu, self.angle_callback)
        rospy.Subscriber('/odomety/filtered', Odometry, self.odom_callback)

        #subscribers at the end to avoid object has no attribute

    def laser_callback(self,msg):
        self.tensor_prime = torch.tensor(msg.ranges).to(device)
        self.num_scans = len(msg.ranges)
        self.angle_max = msg.angle_max*180/math.pi
        self.angle_min = msg.angle_min*180/math.pi 
        self.right_45 = int(3*self.num_scans/8 )
        self.right_90 = int(self.num_scans/4)
        self.left_45 = int(5*self.num_scans/8)
        self.left_90 = int(3*self.num_scans/4)
        self.middle_0 = int(self.num_scans/2)
    def moving_average_filter(self):
        len_queue = 0
        print("Nigil")
        if(self.lidar_queue.full()):
            #print("Hello")
            for i in self.lidar_queue.queue:
                if len_queue == 0:
                    self.tensor = self.tensor_prime
                else:
                    self.tensor = self.tensor.to(device) + i.to(device)
                len_queue = len_queue+1
            self.tensor = self.tensor / len_queue
            _tensor = self.lidar_queue.get()
        else:
            self.tensor = self.tensor_prime
        self.lidar_queue.put(self.tensor_prime, True, 2)

    def depth_callback(self,msg):
        try:
          cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
          print(e)
        
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)
        
    def angle_callback(self,msg):
        
        if(self.initial_angle_set == False):
            _,_,self.yaw_angle = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w] )
            self.initial_yaw = self.yaw_angle
            self.initial_angle_set = True
            print("Initial Angle Set:", self.initial_yaw) 
        else:
            _,_,self.yaw_angle = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w] )
            self.yaw_angle = self.yaw_angle - self.initial_yaw

    def odom_callback(self,msg):

        if(self.initial_odom_set == False):
            self.current_pose_x = msg.pose.pose.position.x
            self.current_pose_y = msg.pose.pose.position.y
            self.initial_pose_x = self.current_pose_x
            self.initial_pose_y = self.current_pose_y
            self.initial_odom_set = True
            print("Initial Odom Set:", self.initial_pose_x, self.initial_pose_y,self.initial_yaw)
        else:
            self.current_pose_x = msg.pose.pose.position.x - self.initial_pose_x
            self.current_pose_y = msg.pose.pose.position.y - self.initial_pose_y


    def get_angles(self,desired_angle, rotation_dir):

        vel_msg = WheelRpm()
        print(f"Desired angle = {desired_angle}")
        print(f"Yaw Angle = {self.yaw_angle}")
 
        if(abs(self.yaw_angle - desired_angle) > 0.05):
            #vel_msg.angular.z = kp_yaw*(desired_angle - 2*self.yaw_angle)
            self.turn_started = True
            vel_msg.omega = rotation_dir*self.turn_speed
            vel_msg.vel = 0
        else:
            self.turn_over =True
            self.turn_started = False
            vel_msg.vel = 0
            vel_msg.omega = 0


        '''
        vel_msg.linear.x = kp*(target_x - self.current_pose_x)*math.cos(self.yaw_angle)+ 3*kp*(target_y - self.current_pose_y)*math.sin(self.yaw_angle)
        if(abs((target_y - self.current_pose_y)/(target_x - self.current_pose_x))>10):
            vel_msg.angular.z = 0
        else:
            vel_msg.angular.z = kp_yaw*(-self.yaw_angle + math.atan((target_y - self.current_pose_y)/(target_x - self.current_pose_x)))
        '''

        return vel_msg
    
    def orient_rover(self):
        vel_msg = WheelRpm()
        a_right = self.tensor[self.right_45:self.middle_0]
        a_left = self.tensor[self.middle_0:self.left_45]
        a_right = a_right <6.0
        a_left = a_left <6.0
        #print(a_left)
        if(self.left_45 == 0 or self.right_45 == 0):
            print("Lidar is not giving points. Please rerun rplidar. ")
            return
        num_left = torch.count_nonzero(a_left)
        num_right = torch.count_nonzero(a_right)
        print(f"Number of points in the right less than 6m = {num_right}")
        print(f"Number of points in the left less than 6m = {num_left}")
        if (num_left-num_right >= self.lr_points):
            vel_msg.omega = self.turn_speed
        elif  (num_right-num_left >= self.lr_points):
            vel_msg.omega = -self.turn_speed
        elif not((num_left == 0) and (num_right == 0)):
            self.aligned_center = True
        print(f"WheelRpm Message given = {vel_msg} in orient_rover")
        self.vel_publisher.publish(vel_msg)



    def check_if_object_in_front(self):
        a_straight = self.tensor[self.right_45:self.left_45]
        #print(self.left_45, self.right_45)
        if(self.left_45 == 0 or self.right_45 == 0):
            print("Lidar is not giving points. Please rerun rplidar. ")
            return
        a = a_straight < 1.5
        print(f"Checking if Object is in Front = {torch.count_nonzero(a)}")
        self.initial_less_than_2m_points = torch.count_nonzero(a)
        if(torch.count_nonzero(a) > 40):
            self.object_in_front =  True
        else:
            self.object_in_front = False



    def move_straight(self, vel, count =0) :

        vel_msg = WheelRpm()
        a_straight = self.tensor[self.right_45:self.left_45]
        #print(self.left_45, self.right_45)
        if(self.left_45 == 0 or self.right_45 == 0):
            print("Lidar is not giving points. Please rerun rplidar. ")
            return vel_msg
        a = a_straight < 1.5
        print(f"Number of points less than 1.5m = {torch.count_nonzero(a)}")
        self.initial_less_than_2m_points = torch.count_nonzero(a)
        if(torch.count_nonzero(a) > 10):
            self.startx = self.current_pose_x
            self.starty = self.current_pose_y
            vel_msg.vel = 0
            print(self.startx, self.starty)
            print("count = ", self.count)
            self.count = self.count + 1
        else:
            vel_msg.vel = self.move_speed

        a_prime = torch.nan_to_num(a_straight, posinf = 500, neginf = 500).to(device)
        avg_distance = torch.mean(a_prime[torch.nonzero(a_prime,as_tuple=True)].float())
        a_with_dist = torch.nonzero(a_prime).to(device)
        print(f"Average Distance = {avg_distance}")
        print(f"Points with distance: {a_with_dist.size(dim=0)} out of {self.num_scans} points")
        print("dimension = ",(a_with_dist.size(dim=0)/self.num_scans)*2*math.pi*avg_distance) #0.61 is multiplied as a constant factor for dimension estimation (?))

        #object_dim = avg_dist*(a_with_dist.shape)/(a_prime.shape)
        #print(object_dim)
        return vel_msg

    def move_to_goal(self):
        vel_msg = WheelRpm()
        if self.count < 20:
            vel_msg = self.move_straight(1)
        elif self.count >=20 and self.object_in_front == True: 
            self.count = 0
            self.turn_started = False
            self.turn_over = False
            self.second_stop = False
            self.third_stop = False
            self.first_stop = False
            self.object_in_front = False
            vel_msg.vel = 0
            vel_msg.omega  = 0
            
        elif self.first_stop == False and self.third_stop == False:
            if(self.turn_over == False): #turning over
                print(self.left_90, self.middle_0)
                a_straight = self.tensor[self.right_45:self.left_90]
                a = a_straight<2.0
                a_prime = torch.nan_to_num(a, posinf = 500, neginf = 500).to(device)
                a_with_dist = torch.nonzero(a_prime).to(device)

                print(f"Turning now. Number of points: {a_with_dist.size(dim=0)}")
                if(a_with_dist.size(dim=0) >10):
                    if(self.turn_timing_count_start == False):
                        self.last_time_turn = time.time()
                        self.turn_timing_count_start = True
                    else:
                        self.time_turn = self.time_turn + time.time()-self.last_time_turn
                        self.last_time_turn = time.time()
                    print(f"Time of turning = {self.time_turn}")
                    vel_msg.omega = self.turn_speed

                elif self.turn_over == False:
                    vel_msg.omega = 0
                    self.turn_over = True
            else:
                a_straight = self.tensor[self.right_90:self.middle_0]
                #print(a_straight)
                a = a_straight<2.5
                a_prime = torch.nan_to_num(a, posinf = 500, neginf = 500).to(device)
                a_with_dist = torch.nonzero(a_prime).to(device)
                self.check_if_object_in_front()
                
                print(f"Second Turn: Number of points less than 2.5m: {a_with_dist.size(dim=0)}")
                print("Second Turn Happening")
                
                if(a_with_dist.size(dim=0)>15):
                    vel_msg.vel = self.move_speed
                    self.displacement_y = self.displacement_y + math.sin(self.yaw_angle)#self.time_turn
                    print(f"Y Displacement = {self.displacement_y}")
                    vel_msg.omega  = 0
                else:
                    self.first_stop = True
                    if(self.displacement_y < 10):
                        self.y_disp_too_less = True
                    self.turn_over  = False
                    vel_msg.vel = 0
                    vel_msg.omega = 0
                    self.turn_timing_count_start = False

        elif self.first_stop == True and self.second_stop ==False and self.third_stop == False:
            self.check_if_object_in_front()
            a_prime = torch.nan_to_num(self.tensor[self.right_90:self.middle_0], posinf = 500, neginf = 500).to(device)
            #print(a_prime)
            dist_to_chk = 1.0
            if(self.y_disp_too_less == True):
                dist_to_chk = 2.0
            a = a_prime < dist_to_chk
            a_with_dist = torch.nonzero(a).to(device)
            print(a_with_dist.size())
            vel_msg = WheelRpm()
            
            if(abs(self.displacement_y) < 20) and self.y_disp_too_less == False:
                self.third_stop = True
            if(self.y_disp_too_less == True) and self.displacement_y > 10:
                self.y_disp_too_less = False
            
            if(a_with_dist.size(dim=0) > 10): #and self.current_pose_y > 0.05):
                vel_msg.vel = self.move_speed
                self.displacement_y = self.displacement_y + math.sin(self.yaw_angle)#self.time_turn
                vel_msg.omega  = self.turn_speed
                if(self.turn_timing_count_start == False):
                    self.last_time_turn = time.time()
                    self.turn_timing_count_start = True
                else:
                    self.time_turn = self.time_turn + time.time()-self.last_time_turn
                    self.last_time_turn = time.time()

                print("Obstacle detected!")
                print(f"Time of turning = {self.time_turn}")
                print(f"Y Displacement = {self.displacement_y}")

                #elif self.current_pose_y > 0.05:
            else:
                vel_msg.vel = self.move_speed
                self.displacement_y = self.displacement_y + math.sin(self.yaw_angle)#self.time_turn
                vel_msg.omega = -self.turn_speed
                if(self.turn_timing_count_start == False):
                    self.last_time_turn = time.time()
                    self.turn_timing_count_start = True
                else:
                    self.time_turn = self.time_turn - time.time()+self.last_time_turn
                    self.last_time_turn = time.time()

                print("No obstacle. Following wall")
                print(f"Time of turning = {self.time_turn}")
                print(f"Y Displacement = {self.displacement_y}")
            
        elif self.third_stop == True:
            if(self.turn_over == False):
                if(self.yaw_angle <0):
                    vel_msg = self.get_angles(0,1)
                else:
                    vel_msg = self.get_angles(0,-1)
                print("Aligning to zero degrees. Turn Over? ", self.turn_over)
            else:
                vel_msg.vel = 0
                vel_msg.omega = 0

            
        else:
            vel_msg.vel = 0
            vel_msg.omega  = 0
        print("Odom, vel_msg", self.current_pose_x, self.current_pose_y, self.yaw_angle,vel_msg)
        self.vel_publisher.publish(vel_msg)
    def spin(self):
        while not rospy.is_shutdown():
            self.moving_average_filter()
            #self.capture_thread.join()
            if self.aligned_center == True:            
                self.move_to_goal()
            else:
                self.orient_rover()
            self.rate.sleep()
        self.exit_signal = True


if __name__ == '__main__':
    try:
        rospy.init_node('go_to_goal', anonymous = True)
        auto = GoToGoal()
        auto.spin()
    except rospy.ROSInterruptException:
        pass
        

