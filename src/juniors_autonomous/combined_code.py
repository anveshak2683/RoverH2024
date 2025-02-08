#!/usr/bin/env python
import copy
import sys
import rospy

# import rosbag
from navigation.msg import gps_data
import math
import time
import cv2
import numpy as np
import imutils
from traversal.msg import WheelRpm
from traversal.srv import *
from std_msgs.msg import Bool
import pyrealsense2 as rs
import threading
import std_msgs.msg as std_msgs
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from ultralytics import YOLO
import cv2
from cv_bridge import CvBridge
import numpy as np
import pyrealsense2 as rs
from ultralytics.utils.plotting import Annotator
from collections import defaultdict

import rospy
import statistics
import numpy as np
import cv2
import open3d as o3d
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

model = YOLO("/home/nvidia/caesar2020/src/juniors_autonomous/best.pt") 

class ZedDepth:
    def __init__(self):
        # Initialize ROS node

        # Subscribers
        # CvBridge for image conversion
        self.bridge = CvBridge()
        self.depth = 0
        # Variables to store latest data
        self.color_image = None
        self.depth_image = None
        self.latest_xmin = 0
        self.latest_ymin = 0
        self.latest_xmax = 0
        self.latest_ymax = 0
        self.annotated_image = None
        self.results= None
        self.first_few = 20

        self.template_r = cv2.imread("Template.png", 0)
        self.template_l = cv2.imread("Template_l.png", 0)
        self.template_r = cv2.resize(self.template_r, (60, 40), cv2.INTER_AREA)
        self.template_l = cv2.resize(self.template_l, (60, 40), cv2.INTER_AREA)
        self.h, self.w = self.template_r.shape
        self.z_angle = self.x_angle = self.y_angle = 0
        self.turn = False
        self.circle_dist = 1.5
        self.dist_thresh = 0.3
        self.angle_thresh = 4
        self.kp = 20
        self.kp_rot = 1.5
        self.kp_straight_rot = 7.5
        self.distance = 10.0
        for i in range(5):
            print("hey! self.distance = 10", self.distance)
        self.direction = "Not Available"
        self.current_latitude = 0.0
        self.current_longitude = 0.0
        self.ret = False
        self.initial_yaw = 0.0
        self.rotate_angle = 90
        self.angles_dict = defaultdict(list)
        self.searchcalled = False
        self.latlong = defaultdict(list)
        self.latlong[0] = "latitude"
        self.latlong[1] = "longitude"
        self.arrow_numbers = 4
        self.gpscalled = 0
        self.depth_image=None
        self.bridge = CvBridge()

        # bag
        #        self.num=i
        #        filename = "imu_data_"+str(self.num)+".bag"
        #        self.bag=rosbag.Bag(filename,'w')
        self.state = False
        self.initial_drift_angle = 0

        # search alg by turning realsense
        self.enc_data = 0
        self.start_time = time.time()
        self.time_thresh = 20
        self.pub = rospy.Publisher("stm_write", std_msgs.Int32MultiArray, queue_size=10)
        self.init = False
        self.start_angle = 55
        self.angle_thresh = 4
        # self.manjari = False
        self.count_arrow = 0
        self.image_avbl = False
        try:
            rospy.Subscriber("state", Bool, self.state_callback)
            print("1")
            rospy.Subscriber("/zed2i/zed_node/imu/data", Imu, self.yaw_callback)
            print("2")
            rospy.Subscriber("enc_auto", std_msgs.Float32MultiArray, self.enc_callback)
            print("3")
            rospy.Subscriber("gps_coordinates", gps_data, self.gps_callback)
            print("4")
            rospy.Subscriber("/zed2i/zed_node/rgb/image_rect_color", Image, self.color_callback)
            print("color")
            rospy.Subscriber("/zed2i/zed_node/depth/depth_registered", Image, self.depth_callback)
            print("depth")
        except KeyboardInterrupt:
            # quit
            sys.exit()
    def state_callback(self, msg):
        self.state = False

    def color_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        try:
            print("entered try1")
            self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            print(self.color_image[0][0])
            self.image_avbl = True
        except Exception as e:
            rospy.logerr("Failed to convert image: %s", e)
            
    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logerr(f"Error in depth_callback: {e}")
     
    
    def get_box(self):
        self.results = model.predict(self.color_image, conf=0.5, max_det=2)
        print("Entered Get Box")
        if self.results!=None:        	
            for r in self.results:
                    self.annotated_image = r.plot()
                    print("Inside For loop")
                    boxes = r.boxes
                    for box in boxes:
                        b = box.xyxy[0]
                        left, top, right, bottom = map(int, b)
                        self.latest_xmin = left
                        self.latest_xmax = right
                        self.latest_ymin = top
                        self.latest_ymax = bottom
                    cv2.imshow('text', self.annotated_image)
                    print("Exited Forloop")
                    self.ret= True
            cv2.waitKey(1)
        else:
            self.ret = False

    

    def arrowdetectmorethan3(self):
        print("Entered process data")
        if self.color_image is None or self.depth_image is None or self.results is None:
            print("returning nothing")
        
        print("x:", (self.latest_xmin + self.latest_xmax) // 2)
        print("y:", (self.latest_ymin + self.latest_ymax) // 2 )
        self.depth = self.depth_image[(self.latest_ymin + self.latest_ymax) // 2, (self.latest_xmin + self.latest_xmax) // 2]
        print(self.depth)
        if math.isnan(self.depth):
            self.ret = False
        arrow_center = (self.latest_xmin + self.latest_xmax) / 2
        return self.ret, "Not Available", arrow_center, self.depth
    
    def move_straight(self):
        msg = WheelRpm()
        msg.hb = False
        if self.init or self.searchcalled:
            print("move_straight() is being ignored due to search().")
            msg.vel = 0
            wheelrpm_pub.publish(msg)
        elif self.ret:
            if abs(self.circle_dist - self.distance) > self.dist_thresh:
                if self.distance != 0.0 and self.distance != 2.5:
                    msg.vel = max(
                        27, int(0 + self.kp * (self.circle_dist - self.distance))
                    )
                    print("Moving straight. ", (self.circle_dist - self.distance))
                    wheelrpm_pub.publish(msg)
                else:
                    msg.vel = 30
                    wheelrpm_pub.publish(msg)

            else:
                msg.vel = 0
                msg.omega = 0
                wheelrpm_pub.publish(msg)
                print("Stopped going Straight")
                self.gpscalled = 1
                gps_data_pub.publish(self.gpscalled)
                '''
                for i in range(100):
                    gps_data_pub.publish(self.gpscalled)
                    rate.sleep()  # 10s   # Competition rules say 10s
                '''
                if self.count_arrow <= self.arrow_numbers:
                    # rospy.sleep(10)
                    # self.latlong[0].append(msg.latitude)
                    # self.latlong[1].append(msg.longitude)
                    self.gpscalled = 0
                    print()
                    print("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
                    print("lat:", self.current_latitude)
                    print("long:", self.current_longitude)
                    print()
                    self.turn = True
                    self.rotate_angle = 90
                    self.initial_yaw = self.z_angle
                    self.count_arrow += 1
                else:
                    self.v1_competition()

        #                self.write_coordinates()

        else:
            print("Forward")
            msg.vel = 25
            wheelrpm_pub.publish(msg)
            self.turn = False

    def process_dict(self):
        print("self.searchcalled (should print true always):", self.searchcalled)
        if not self.init and self.searchcalled:
            # the first part is not needed, because whenever self.searchcalled is set to true, self.init is set to false
            # actually, this if is only not needed, because in main(), there is an if self.searchcalled(), then process_dict(), which takes care of everything

            print("the dictionary with dist:[enc angles] :- ", self.angles_dict)
            self.searchcalled = False
            max_length_key = 0.0

            try:
                print("Entered Try of proces_dict")
                # if min(self.angles_dict.keys()) == '0.0':
                # del self.angles_dict['0.0']
                if min(self.angles_dict.keys()) != 0:
                    print("Entered if")
                    # del self.angles_dict['0.0']
                    max_length_key = min(self.angles_dict.keys())
                    # max_length_key = max(self.angles_dict, key=lambda k: len(self.angles_dict[k]))
                    self.min_dist = max_length_key
            except:
                print("The list is empty. No minimum value.")
                self.init = False
                self.searchcalled = False
                return
            if len(self.angles_dict[max_length_key + 1]) != 0:
                self.which_enc_angle_to_turn = (
                    sum(self.angles_dict[self.min_dist])
                    + sum(self.angles_dict[self.min_dist + 1])
                ) / (
                    len(self.angles_dict[self.min_dist])
                    + len(self.angles_dict[self.min_dist + 1])
                )
            else:
                self.which_enc_angle_to_turn = sum(
                    self.angles_dict[self.min_dist]
                ) / len(self.angles_dict[self.min_dist])
            print("Angle to turn:", self.which_enc_angle_to_turn)
            print("the dictionary with dist:[enc angles] :- ", self.angles_dict)

            # encoder need not be perfect. if in case there is some cup, edit this angle as per your needs
            if self.which_enc_angle_to_turn < 0:
                self.direction = "left"
                # self.rotate_angle=abs(self.which_enc_angle_to_turn + 0.5*self.angle_thresh) #+2 degrees
                self.rotate_angle = abs(self.which_enc_angle_to_turn)
            else:
                self.direction = "right"
                # self.rotate_angle=(self.which_enc_angle_to_turn - 0.5*self.angle_thresh) #+2 degrees
                self.rotate_angle = abs(self.which_enc_angle_to_turn)

            self.turn = True
            self.initial_yaw = self.z_angle
            self.angles_dict = defaultdict(list)

            # self.init = False
    def rotate(self, dir):
        print("Rotation has been called")
        msg = WheelRpm()
        msg.vel = 0
        msg.hb = False
        diff = self.z_angle - self.initial_yaw
        if diff > 120:
            diff = diff - 360
        elif diff < -120:
            diff = diff + 360
        print("diff=", diff)
        """
        if (-60<self.z_angle-self.initial_yaw<60):
            error=30
        else:
            error = 90-abs(self.z_angle-self.initial_yaw)
        """
        print("Rotation angle:", self.rotate_angle)
        print("Initial Yaw:", self.initial_yaw)
        print("Current z angle", self.z_angle)
        error = self.rotate_angle - abs(diff)
        print("error=", error)
        # if self.direction == -1:
        #   self.rotate_angle = self.rotate_angle +2
        if abs(error) >= 0.5 * self.angle_thresh:
            msg.omega = 0 + (dir * 40)
            msg.vel = 25
            print("Calling Rotate, printing Z angle below")
            print(error)
            wheelrpm_pub.publish(msg)
        else:
            msg.omega = 0
            msg.vel = 0
            wheelrpm_pub.publish(msg)
            # self.z_angle=0

            self.initial_drift_angle = self.z_angle
            print("****ROTATE DONE*****")
            # self.distance = 10.0
            self.start_time = time.time() - 10
            self.turn = False
            self.direction = "Not Available"
            #rospy.sleep(2)

    def search(self):
        print("==========================================================================================================SEARCH START=====================================================================================")
        print("self.searchcalled (should print false here always):", self.searchcalled)
        if abs(self.enc_data) < 0.6 * self.angle_thresh and self.ret and not self.init:
            # if arrow is detected and realsense is facing straight, then come out of search immediately.
            return

        print("Search() has been called.")
        # self.searchcalled = True
        print("time.time():", time.time())
        print("self.start_time:", self.start_time)
        if time.time() - self.start_time < self.time_thresh:  # time_thresh is 20s
            print(
                "time.time()-self.start_time (when this becomes 20s, search will happen):",
                time.time() - self.start_time,
            )
            return
        msg1 = WheelRpm()
        msg1.hb = False
        msg1.omega = 0
        msg1.vel = 25
        wheelrpm_pub.publish(msg1)
        print("Rover has stopped.")
        msg = std_msgs.Int32MultiArray()
        msg.data = [0, 0, 0, 0, 0, 0]

        msg.layout = std_msgs.MultiArrayLayout()
        msg.layout.data_offset = 0

        msg.layout.dim = [std_msgs.MultiArrayDimension()]
        msg.layout.dim[0].size = msg.layout.dim[0].stride = len(msg.data)
        msg.layout.dim[0].label = "write"
        self.pub.publish(msg)

        print("Entered while loop.")
        while (
     
            self.init == False
            and abs(self.enc_data) < abs(self.start_angle) - 2 * self.angle_thresh
        ):
            # to make the realsense go to the 60 degree maximum before starting the burst search
            print(self.enc_data)
            print("meaoo")
            msg.data = [0, 255, 0, 0, 0, 0]
            rate.sleep()
            self.pub.publish(msg)
            self.start_time = time.time() - self.time_thresh
        msg.data = [0, 0, 0, 0, 0, 0]
        print("Exited while loop.")
        self.init = True
        print("self.init (set to true in the previous line:", self.init)
        print("Realsense's angle:", self.enc_data)
        print("self.ret:", self.ret)
        if (
            self.init == True
            and self.enc_data > -60
            and not self.ret
        ):
            # if arrow is not detected and the realsense has not gone beyond the 60 degree maximum, continue moving realsense
            # self.init is not made false, so this will happen again when search is called in main()

            print("Camera Moving")
            msg.data = [0,-255,0,  0, 0, 0]
            rate.sleep()
            self.pub.publish(msg)
            print()
            # main area
        elif (
            self.init == True
            and abs(self.enc_data) < abs(self.start_angle)
            and self.ret
        ):
            # if arrow is detected and the realsense is within the 60 degree maximum, append the arrow's values and continuemoving realsense
            # self.init is not made false, so this will happen again when search is called in main()

            # self.distance = self.distance*1
            # self.distance = self.distance // 1
            # self.distance = self.distance / 1
            self.distance = float(round(self.distance))
            if self.distance < 0.0:  # change
                self.angles_dict[self.distance].append(self.enc_data)
                print("I'm appending to angles_dict")
            msg.data = [0,-255, 0, 0, 0, 0]
            self.pub.publish(msg)
            rate.sleep()
            print("Arrow found at: ", self.enc_data)
            print()
        elif not self.ret:
            # when the realsense has crossed the 60 degree maximum, realsense comes back to middle and the dictionary is processed
            # self.init is set to false (when next search() is called, realsense will first move to the 60 degree maximum)
            # and the counting of time is reset (that is, the next search will happen at least 20s after this block of code)

            while abs(self.enc_data) > self.angle_thresh:
                if self.enc_data > 0:
                    msg.data = [0,-255, 0, 0, 0, 0]
                else:
                    msg.data = [0,255, 0, 0, 0, 0]
                rate.sleep()
                self.pub.publish(msg)
            msg.data = [0, 0, 0, 0, 0, 0]
            self.pub.publish(msg)
            self.init = False
            self.searchcalled = True
            self.distance = 10.0
            self.start_time = time.time()
        """     
            while self.enc_data >5:
                pass
                #go in one direction to 0.
            while self.enc_data < -5:
                pass
                #go in other direction to 0.
            return
        """
        print("==========================================================================================================SEARCH END=====================================================================================")
    def quaternion_to_euler(self,x, y, z, w):

# Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

# Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
            siny_cosp = 2 * (w * z + x * y)
            cosy_cosp = 1 - 2 * (y * y + z * z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
        #print("current yaw", yaw)
        return roll, pitch, yaw

    def yaw_callback(self,data):
        current_x = data.orientation.x
        current_y = data.orientation.y
        current_z = data.orientation.z
        current_w = data.orientation.w
        current_tuple=(current_x, current_y, current_z, current_w)
        current_pitch, current_roll, self.z_angle = self.quaternion_to_euler(data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w)

        if self.initial_yaw == 0:
            self.initial_yaw = self.z_angle*180/math.pi
        self.z_angle = self.z_angle*180/math.pi - self.initial_yaw
        #print("self.z_angle in callback", self.z_angle)

        if self.z_angle < -179:
            self.z_angle = self.z_angle + 360
        elif self.z_angle > 179:
            self.z_angle = self.z_angle - 360

    def enc_callback(self, msg):
        #print(msg.data)
        #print("meao")
        self.enc_data = msg.data[0]

    def gps_callback(self, msg):
        if msg.latitude and msg.longitude:
            self.current_latitude = msg.latitude
            self.current_longitude = msg.longitude
            file_object = open("coordinates.txt", "w")
            file_object.write(
                "latitude :%f, longitude :%f", msg.latitude, msg.longitude
            )
            file_object.close()
    def main(self):
        gps_data_pub.publish(self.gpscalled)

        if (
            self.count_arrow == self.arrow_numbers
        ):  # change before competition  #if we take intervention, change this number acc to your needs
            print("Im in if block of main")
            if (not self.turn and not self.ret) or (self.init):
                self.search()
                if self.searchcalled:
                    self.process_dict()

            if not self.turn:
                self.ret, direction, pix, self.distance = self.cone_model()
                if self.ret == True:
                    print("Cone detected")
                    print("self.cone_distance:", self.distance)

                else:
                    print("Still searching")
                    self.ret = False
                self.move_straight()
            else:  # if self.turn is true, everything stops and rover does only turning
                print("Im going into rotate block in main")
                if self.direction == "left":
                    print("rotating left")
                    self.rotate(1)
                else:
                    print("rotating right")
                    self.rotate(-1)

        else:
            print("Im in else block of main")
            if (not self.turn and not self.ret) or (self.init):
                self.search()
                print("self.distance:", self.distance)
                if self.searchcalled:
                    self.process_dict()
            if (
                not self.turn
            ):  # this is there because detection need not happen when turning
                # note that self.turn is made true once in process_dict(), so we put this if condition again.

                self.ret, direction, pix, self.distance = self.arrowdetectmorethan3()
                print(f"Depth = {self.distance}, center = {pix}")
                if self.distance < 3.0:
                    ret, depth_frame, color_frame, depth = self.get_frame()
                    ret, direction, pix, distance = self.arrowdetectlessthan3(
                        color_frame, depth
                    )
                    # if not self.ret:
                    #     self.ret,self.direction,pix,self.distance=self.arrowdetectmorethan3()
                    if ret == True:
                        self.ret = ret
                        self.direction = direction
                        self.distance = distance

                if self.ret:
                    print("arrow detected at distance: " + str(self.distance))
                    print("Direction: " + self.direction)
                else:
                    print("Trying to detect arrow...")
                self.move_straight()

            else:  # if self.turn is true, everything stops and rover does only turning
                print("Im going into rotate block in main")
                if self.direction == "left":
                    print("rotating left")
                    self.rotate(1)
                else:
                    print("rotating right")
                    self.rotate(-1)
    def run(self):
        # Main loop
        rate = rospy.Rate(10) 
        while not rospy.is_shutdown():
            print("goin in")
            if(self.image_avbl):
                self.get_box()
                self.main()
            rate.sleep()

if __name__ == '__main__':
    try:

        rospy.init_node('zed_depth', anonymous=True)
        rate = rospy.Rate(10) 
        wheelrpm_pub = rospy.Publisher("motion", WheelRpm, queue_size=10)
        gps_data_pub = rospy.Publisher("gps_bool", std_msgs.Int8, queue_size=10)
        fusion_node = ZedDepth()
        fusion_node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()


