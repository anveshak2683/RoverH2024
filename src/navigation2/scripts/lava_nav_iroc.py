


import rospy
import math
import numpy as np 
import cv2
from traversal.msg import WheelRpm
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout, Bool, Float32
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from tf.transformations import euler_from_quaternion



class habibo_legacy():
    def __init__(self):
        self.initial_yaw = 0
        self.initial_odom = [0, 0]
        self.current_odom = [0,0]
        self.odom_initialized = False
        self.color_image = []
        self.current_pitch = self.current_roll = self.current_yaw = 0.0
        self.initial_pitch = self.initial_roll = self.initial_yaw = 0.0
        self.goals_x = [0.9, 2.45, 2.45, 5.2, 6.22]
        self.goals_y = [1.0, 1.8, 1.5, 2.2, 1.55]
        self.bridge = CvBridge()
        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_callback)
        rospy.Subscriber("/zed2i/zed_node/rgb/image_rect_color",
                         Image, self.image_callback)
        rospy.Subscriber("/zed2i/zed_node/depth/depth_registered",
                         Image, self.depth_callback)
        rospy.Subscriber("/zed2i/zed_node/imu/data", Imu, self.imu_callback)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.omega = 0
        self.pixel_dict = defaultdict(list)
        self.left_check = self.right_check = self.front_check = 0
        self.front_min = 1000000
        

    def odom_callback(self,data):
        self.current_odom[0] = data.pose.pose.position.x
        self.current_odom[1] = data.pose.pose.position.y
        if self.odom_initialized = False:
            self.initial_odom[0] = self.current_odom[0]
            self.initial_odom[1] = self.initial_odom[1]
            self.odom_initialized = True
        self.current_odom[0] = self.current_odom[0] - self.initial_odom[0]
        self.current_odom[1] = self.current_odom[1] - self.initial_odom[1]

    def image_callback(self, data):
        cframe = data
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(cframe, "bgr8")
        except Exception as e:
            print(e)

    def depth_image(self, data):
        dframe = data
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(dframe, "passthrough")
        except Exception as e:
            print(e)

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

    def laser_callback(self, data):
        tmp = np.nan_to_num(data.ranges, copy = True, nan = 0.8, posinf = 1000, neginf = 10000)
        front_check_sum = 0
        left_check_sum = 0
        right_check_sum = 0
        length = len(data.ranges)
        left_start = 0.25*length
        right_start = 0.75*length
        front_start = 0.45*length
        front_min_1 = 100

        for i in range(50):
            left_check_sum = left_check_sum + data.ranges[left_start + i]
            right_check_sum = right_check_sum + data.ranges[right_start - i]
            front_check_sum = front_check_sum + data.ranges[front_start + i]
            if front_min_1 > data.ranges[front_start + i]:
                front_min_1 = data.ranges[front_start + i]
                self.front_min = front_start + i
        self.left_check = left_check_sum / 50
        self.right_check = right_check_sum / 50
        self.front_check = front_check_sum / 50


    def obstacle_avoidance(self):
        thresh = 2
        msg = WheelRpm()
        direction = 0
        if self.left_check < thresh:
            direction = 1
        elif self.right_check < thresh:
            direction = -1
        elif self.front_check < thresh:
            if 

        

    




