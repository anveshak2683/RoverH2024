#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32
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
import cmath
from navigation2.msg import red

class GoToGoal():
    def __init__(self):
        self.goals_x = [3.5,11.5,9.9,11.5,8,11.5,8.83]
        self.goals_y = [6.5,8.3,1.35,8.3,6,8.3,8.3]
        #self.vel_publisher = rospy.Publisher('/motion', WheelRpm, queue_size = 10)
        self.obstacle_detected_pub = rospy.Publisher('/pranav_red', red, queue_size = 10)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        rospy.Subscriber('/goal_ik',Int32,self.ik_red_callback)
        rospy.Subscriber('/goal_blue',Int32,self.ik_blue_callback)
        self.rate = rospy.Rate(10)
        self.current_pose_x = 0
        self.current_pose_y = 0
        self.yaw_angle = 0
        self.yaw_angle_prime = 0
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
        self.target_dist = 0
        self.num_scans = 0
        self.angle_max = 0 #for lidar
        self.angle_min = 0 #for lidar
        self.left_45 = 0 #for lidar
        self.left_90 = 0
        self.right_90 = 0
        self.right_45 = 0 #for lidar
        self.middle_0 = 0
        self.turn_speed = -50
        self.move_speed = 40
        self.goal_counter = 0
        self.time_turn = 0 #to keep a track of turn timing
        self.turn_timing_count_start = False
        self.last_time_turn = 0
        self.displacement_y = 0 #to basially estimate odom with time
        self.obstacle_detected = False
        self.initial_odom_set = False
        self.initial_angle_set = False

        self.initial_yaw = 0
        self.initial_pose_x = 0
        self.initial_pose_y = 0
        self.initial_angle_prime = 0
        ##SPL VARIABLE##
        self.y_disp_too_less = False
        #below code is for orienting rover towards object
        self.aligned_center = False
        self.lr_points = 10
        
        ####VARIABLES FOR DISABLING OBSTACLE AVOIDANCE##########3
        self.ik_start_tanish = 0
        self.ik_start_anuj = 0
        self.ik_start_tanish_prev=0
        self.ik_start_anuj_prev=0

        #rospy.Subscriber('/intel/depth/image_raw', Image, self.depth_callback)
        rospy.Subscriber('/robot/dlo/odom_node/odom', Odometry, self.odom_callback)

        #subscribers at the end to avoid object has no attribute
    
    def ik_red_callback(self,msg):
        self.ik_start_tanish = msg.data
        #if self.ik_start_tanish-self.ik_start_tanish_prev==-1:
            #self.goal_counter+=1
#        else:
  #          pass
   #     self.ik_start_tanish_prev=self.ik_start_tanish


    def ik_blue_callback(self,msg):
        self.ik_start_anuj=msg.data
     #   if self.ik_start_tanish-self.ik_start_tanish_prev==-1:
      #      self.goal_counter+=1
       # else:
        #    pass
        #self.ik_start_tanish_prev=self.ik_start_tanish


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

        if(self.initial_angle_set == False):
            _,_,self.yaw_angle = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w] )
            self.initial_yaw = self.yaw_angle
            self.initial_angle_prime = self.yaw_angle
            self.initial_angle_set = True
            print("Initial Angle Set:", self.initial_yaw) 
        else:
            _,_,self.yaw_angle = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w] )
            self.yaw_angle = self.yaw_angle - self.initial_yaw

    def get_angles(self, desired_angle, rotation_dir):
        

        vel_msg = WheelRpm()
        print(f"Desired angle = {desired_angle}")
        print(f"Yaw Angle = {self.yaw_angle}")
 
        if(abs((self.yaw_angle-self.initial_angle_prime) - desired_angle) > 0.05):
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
    
    def check_if_object_in_front(self):
        a_straight = self.tensor[self.right_45:self.left_45]
        #print(self.left_45, self.right_45)
        if(self.left_45 == 0 or self.right_45 == 0):
            print("Lidar is not giving points. Please rerun rplidar. ")
            return
        a = a_straight < 1.2 #this is for multiple obstacle avoidance
        print(f"Checking if Object is in Front = {torch.count_nonzero(a)}")
        self.initial_less_than_2m_points = torch.count_nonzero(a)
        if(torch.count_nonzero(a) > 40):
            self.object_in_front =  True
        else:
            self.object_in_front = False



    def move_straight(self, vel, count =0) :

        kp = 10
        kp_rot = 0.8
        kp_linear = 25
        # goal_vec = np.array(2)
        # print("goal_vec:",goal_vec)
        goal_vec = [self.goals_x[self.goal_counter], self.goals_y[self.goal_counter]]
        print("goal_vec: ", goal_vec)
        cn = complex((goal_vec[0]-self.current_pose_x), (goal_vec[1]-self.current_pose_y))
        self.desired_angle_gtg = cmath.phase(cn)
        
        print("desired turn in gtg", self.desired_angle_gtg)
        goal_distance = math.sqrt(
            ((goal_vec[0] - self.current_pose_x)**2) + (goal_vec[1] - self.current_pose_y)**2)
        print("goal distance in metres", goal_distance)
        print("self.goal_counter", self.goal_counter)
        omega = int(kp*(self.yaw_angle - self.desired_angle_gtg))
        desired_vel = int(kp_linear * goal_distance)
        g = WheelRpm()
        g.vel = min(desired_vel, self.move_speed)

        if abs(self.yaw_angle - self.desired_angle_gtg) > 6*math.pi/180:
            print("self.omega gtg", omega)
            if omega < 0:
                g.omega = int(min(self.turn_speed,int(min(kp_rot*(self.yaw_angle - self.desired_angle_gtg), 20))))
                g.vel = 0
            else:
                g.omega = int(max(-self.turn_speed,int(max(kp_rot*(self.yaw_angle - self.desired_angle_gtg), -20))))
                g.vel = 0
        elif abs(self.yaw_angle - self.desired_angle_gtg) < 6*math.pi/180:
            print("self.omega gtg", 0)
            self.initial_angle_prime = self.yaw_angle
            g.omega = 0
        vel_msg = WheelRpm()
        a_straight = self.tensor[self.right_45:self.left_45]
        if(goal_distance < 0.4):
            print(f"Goal {self.goal_counter} reached!")
            self.goal_counter +=1 #increment, since goal is reached
        #print(self.left_45, self.right_45)
        if(self.left_45 == 0 or self.right_45 == 0):
            print("Lidar is not giving points. Please rerun rplidar. ")
            return vel_msg
        a = a_straight < 1.0 #first obstacle avoidance
        print(f"Number of points less than 1.0m = {torch.count_nonzero(a)}")
        self.initial_less_than_2m_points = torch.count_nonzero(a)
        if(torch.count_nonzero(a) > 10 and self.ik_start_tanish == 0 and self.ik_start_anuj == 0):
            self.startx = self.current_pose_x
            self.starty = self.current_pose_y
            g.vel = 0
            g.omega = 0
            print(self.startx, self.starty)
            print("count = ", self.count)
            self.count = self.count + 1
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False
        #object_dim = avg_dist*(a_with_dist.shape)/(a_prime.shape)
        #print(object_dim)
        return g

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
            self.obstacle_detected = True
            vel_msg.vel = 0
            vel_msg.omega  = 0
            
        elif self.first_stop == False and self.third_stop == False:
            if(self.turn_over == False): #turning over
                a_straight = self.tensor[self.right_45:self.middle_0] #range of points for checking first turn
                a = a_straight<1.2
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
                a = a_straight<1.2
                a_prime = torch.nan_to_num(a, posinf = 500, neginf = 500).to(device)
                a_with_dist = torch.nonzero(a_prime).to(device)
                self.check_if_object_in_front()
                
                print(f"Second Turn: Number of points less than 2.5m: {a_with_dist.size(dim=0)}")
                print("Second Turn Happening")
                
                if(a_with_dist.size(dim=0)>15):
                    vel_msg.vel = self.move_speed
                    self.displacement_y = self.displacement_y + math.sin(self.yaw_angle - self.initial_angle_prime)#self.time_turn
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
            vel_msg = WheelRpm()
            
            if(abs(self.yaw_angle - self.initial_angle_prime) < 10*math.pi/180) and self.y_disp_too_less == False: #changed to angle threshold
                self.third_stop = True
            if(self.y_disp_too_less == True) and self.displacement_y > 10:
                self.y_disp_too_less = False
            
            if(a_with_dist.size(dim=0) > 10): #and self.current_pose_y > 0.05):
                vel_msg.vel = self.move_speed
                self.displacement_y = self.displacement_y + math.sin(self.yaw_angle - self.initial_angle_prime)#self.time_turn
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
                self.displacement_y = self.displacement_y + math.sin(self.yaw_angle - self.initial_angle_prime)#self.time_turn
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
                if((self.yaw_angle-self.initial_angle_prime) <0):
                    vel_msg = self.get_angles(0,1)
                else:
                    vel_msg = self.get_angles(0,-1)
                print("Aligning to zero degrees. Turn Over? ", self.turn_over)
            else:
                self.count = 0
                self.turn_started = False
                self.turn_over = False
                self.second_stop = False
                self.third_stop = False
                self.first_stop = False
                self.object_in_front = False
                vel_msg.vel = 0
                vel_msg.omega  = 0
                self.displacement_y = 0
                self.obstacle_detected = False
        else:
            vel_msg.vel = 0
            vel_msg.omega  = 0
        print(f"Odom = ({self.current_pose_x}, {self.current_pose_y}), Velocity given = {vel_msg.vel}, Omega given = {vel_msg.omega}, Yaw Angle = {self.yaw_angle}")
        #self.vel_publisher.publish(vel_msg)
        red_value = red()
        red_value.vel = vel_msg.vel
        red_value.omega = vel_msg.omega
        red_value.detect = self.obstacle_detected
        self.obstacle_detected_pub.publish(red_value)
    def spin(self):
        while not rospy.is_shutdown():
            self.moving_average_filter()
            self.move_to_goal()
            self.rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('go_to_goal', anonymous = True)
        auto = GoToGoal()
        auto.spin()
    except rospy.ROSInterruptException:
        pass
        

