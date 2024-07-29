#!/usr/bin/env python3
import sys
import rospy
#import rosbag  
#from navigation.msg import gps_data
import math
import time
import cv2
import numpy as np
import imutils
from traversal.msg import WheelRpm
from traversal.srv import *
from std_msgs.msg import Bool,Int32
#import pyrealsense2 as rs
#import threading
import std_msgs.msg as std_msgs
from ultralytics import YOLO
import cv2
import numpy as np
#import pyrealsense2 as rs
from ultralytics.utils.plotting import Annotator
from collections import defaultdict
from collections import OrderedDict
from cv2 import aruco
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge, CvBridgeError
#from PIL import Image as Image2
from tf.transformations import euler_from_quaternion



class Aruco_detection():

    def __init__(self):

        #This is for realsense feed
        # self.pipeline = rs.pipeline()
        # config = rs.config()
        # pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        # pipeline_profile = config.resolve(pipeline_wrapper)
        # device = pipeline_profile.get_device()
        # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        # config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        # self.pipeline.start(config)
        

        self.distance = 100000 #random default distance, was undeclared before. 

        self.distance_list = []
        self.initial_yaw = 0
        self.where_am_i = None
        self.enc_data = 0
        self.start_angle = 60
        self.start_angle_1 = 90
        self.start_time = time.time()
        self.angles_dict = defaultdict(list)
        self.tmp_dict = defaultdict(list)
        self.angle_thresh = 2
        self.z_angle = 0
        self.color_image = np.zeros((360,640))
        self.depth_image = np.zeros((360,640))
        self.initial_drift_angle = 0
        self.temp_z = 0
        self.arrest_motion = False

        #variables used for controlling something

        self.ret = False
        self.init = False
        self.searchcalled = False #variable to control when and if to enter process_dict()

        self.turn = False
        self.state = True
        self.code_stopping_variable = False #literally what the name saysi
        self.rotate_start= False

        self.check_msg=Int32()
        self.check_msg.data=0

        self.execute_goal=False #check condition variable
        self.counter = 0
        self.led_published = False


        self.pub = rospy.Publisher('stm_write', std_msgs.Int32MultiArray, queue_size=10)
        self.task_completed_pub = rospy.Publisher('/task_completed', Int32, queue_size=100)
        self.check_publisher=rospy.Publisher('/check',Int32,queue_size=100)
        self.led_pub=rospy.Publisher('/rover_state',Int32,queue_size=10)

        try:
            rospy.Subscriber('state', Bool, self.state_callback)
            print("1")
            rospy.Subscriber('imu',Imu,self.yaw_callback)
            print("2")
            rospy.Subscriber('enc_auto',std_msgs.Float32MultiArray,self.enc_callback)
            print("3")  
#            rospy.Subscriber('gps_coordinates', gps_data, self.gps_callback)
            print("4")
            rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback)
            print('5')
            rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)
            print('6')
            rospy.Subscriber('/check',Int32,self.check_callback)
            #rospy.Subscriber('/code_running', Bool, self.code_callback)
            #print('7')
        except KeyboardInterrupt:
            # quit
            sys.exit()

    def check_callback(self,msg):
        print("check callback hit")
        print()
        if msg.data==1:
            self.execute_goal=True
        elif msg.data==2:
            self.execute_goal=False

    def color_callback(self, data):
        try:
            bridge = CvBridge()
            self.color_image = bridge.imgmsg_to_cv2(data, 'bgr8')
#            self.color_image = PIL.Image2.convert(mode ="RGB")
        except CvBridgeError as e:
            print(e)
    
    def depth_callback(self, data):
        try:
            bridge = CvBridge()
            self.depth_image = bridge.imgmsg_to_cv2(data, 'passthrough')
        except CvBridgeError as e:
            print(e)
    
    # def get_frame(self):    #to get data from realsense
    #     frames = self.pipeline.wait_for_frames()
    #     depth_frame = frames.get_depth_frame()
    #     color_frame = frames.get_color_frame()
    #     align=rs.align(rs.stream.color)
    #     frames=align.process(frames)
    #     depth_image = np.asanyarray(depth_frame.get_data())
    #     color_image = np.asanyarray(color_frame.get_data())
    #     cv2.imshow("Aruco Marker Detection",color_image)
    #     if not depth_frame or not color_frame:
    #         return False, None, None, None
    #     return True, depth_image, color_image, depth_frame

    def aruco_recognition(self,color_image, depth_image):    #can give ID, angle of tilt of aruco with rover, self.ret = True and self.distance      #opencv doesn't work for long distances
        # define an empty custom dictionary with 
        self.aruco_dict = aruco.Dictionary(0, 5, 1)
        # add empty bytesList array to fill with 3 markers later
        self.aruco_dict.bytesList = np.empty(shape = (10, 4, 4), dtype = np.uint8)
        # add new marker(s)
        #mybits = np.array([[1,0,0,0,0],[1,0,0,0,0],[0,1,0,0,1],[0,1,0,0,1],[1,0,0,0,0]], dtype = np.uint8)
        #self.aruco_dict.bytesList[0] = aruco.Dictionary_getByteListFromBits(mybits)
        #mybits = np.array([[1,0,0,0,0],[1,0,0,0,0],[0,1,1,1,0],[0,1,0,0,1],[0,1,0,0,1]], dtype = np.uint8)
        #self.aruco_dict.bytesList[1] = aruco.Dictionary_getByteListFromBits(mybits)
        mybits = np.array([[1,0,0,0,0],[1,0,1,1,1],[1,0,0,0,0],[1,0,0,0,0],[0,1,1,1,0]], dtype = np.uint8)
        self.aruco_dict.bytesList[0] = aruco.Dictionary_getByteListFromBits(mybits)
        #mybits = np.array([[1,0,0,0,0],[1,0,1,1,1],[1,0,0,0,0],[0,1,1,1,0],[0,1,0,0,1]], dtype = np.uint8)
        #self.aruco_dict.bytesList[3] = aruco.Dictionary_getByteListFromBits(mybits)
        #mybits = np.array([[1,0,0,0,0],[1,0,0,0,0],[0,1,0,0,1],[0,1,1,1,0],[1,0,0,0,0]], dtype = np.uint8)
        #self.aruco_dict.bytesList[4] = aruco.Dictionary_getByteListFromBits(mybits)
        mybits = np.array([[1,0,1,1,1],[1,0,0,0,0],[0,1,0,0,1],[0,1,0,0,1],[1,0,1,1,1]], dtype = np.uint8)
        self.aruco_dict.bytesList[1] = aruco.Dictionary_getByteListFromBits(mybits)
        #mybits = np.array([[1,0,1,1,1],[1,0,1,1,1],[1,0,1,1,1],[0,1,0,0,1],[1,0,0,0,0]], dtype = np.uint8)
        #self.aruco_dict.bytesList[6] = aruco.Dictionary_getByteListFromBits(mybits)
        #mybits = np.array([[1,0,0,0,0],[0,1,0,0,1],[1,0,1,1,1],[1,0,1,1,1],[0,1,0,0,1]], dtype = np.uint8)
        #self.aruco_dict.bytesList[7] = aruco.Dictionary_getByteListFromBits(mybits)
        mybits = np.array([[1,0,1,1,1],[1,0,0,0,0],[1,0,0,0,0],[0,1,1,1,0],[1,0,1,1,1]], dtype = np.uint8)
        self.aruco_dict.bytesList[2] = aruco.Dictionary_getByteListFromBits(mybits)
        #mybits = np.array([[1,0,1,1,1],[0,1,1,1,0],[1,0,0,0,0],[0,1,1,1,0],[1,0,0,0,0]], dtype = np.uint8)
        #self.aruco_dict.bytesList[9] = aruco.Dictionary_getByteListFromBits(mybits)
        # save marker images
        # for i in range(len(self.aruco_dict.bytesList)):
        #     cv2.imwrite("custom_aruco_" + str(i) + ".png", aruco.generateImageMarker(self.aruco_dict, i, 128))
        intrinsic_camera=np.array(((607.7380981445312, 0.0, 325.2829284667969),(0.0, 606.8139038085938, 238.5009307861328),(0,0,1.0)))
        distortion=np.array((0.0, 0.0, 0.0, 0.0, 0.0))
        
        print(color_image.shape)
        print()
        print("inside aruco_recognition")
        depth_array = None
        ret, output, ids, depth_array = self.pose_estimation(color_image, depth_image, intrinsic_camera, distortion)        #returns a list of two entities at all times
        print("depth_array inside recognition", depth_array)
        if len(depth_array)!= 0:
            depth_array.sort()
            while (depth_array[0]==0):
                depth_array.pop(0)
#                depth_array.remove(0)
            if len(depth_array)>=2:                                  #filters used are minimum distance and not zero(if something else pops up during testing, add here)
                depth_array = [depth_array[0], depth_array[1]]
            elif len(depth_array)==1:
                depth_array = [depth_array[0], None]
            else:
                depth_array = [None, None]
        return ret, ids, depth_array

    def pose_estimation(self,frame,depth_frame, matrix_coefficients, distortion_coefficients):      #pose estimation means to find the tilt of aruco with the rover     #probably won't be used in this code
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        parameters = aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(self.aruco_dict,parameters)
        corners, ids, _ = detector.detectMarkers(gray)
        print(f"Pose Estimation Called, ids are {ids}")
        ret = False
        depth_array = []
        rotate_angle = 0
        if ids is not None and len(ids) > 0:
            object_points = np.array([[0, 0, 0],
                                    [0, 1, 0],
                                    [1, 1, 0],
                                    [1, 0, 0]], dtype=np.float32)
            for i in range(min(len(ids), 2)):
                ret = True
                _, rvecs, tvecs = cv2.solvePnP(object_points, corners[i][0],matrix_coefficients,distortion_coefficients)
                # Draw detected markers
                aruco.drawDetectedMarkers(frame.copy(), corners)
                # Draw coordinate axes
                axis_length = 0.1  # Length of the axis in meters
                axis_points = np.float32([[0,0,0], [axis_length,0,0], [0,axis_length,0], [0,0,-axis_length]]).reshape(-1,3)
                image_points, _ = cv2.projectPoints(axis_points, rvecs, tvecs, matrix_coefficients, distortion_coefficients)
                frame = cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvecs, tvecs, axis_length)
                frame = aruco.drawDetectedMarkers(frame, corners, ids)
                #print(f"Rvecs = {rvecs}, Tvecs = {tvecs}")
                #print(f"Corners = {corners}")
                
                a=self.rotation_vector_to_euler_angles(rvecs,tvecs)
                #print(f"a = {a}")

                if a[0][0]<0:
                    angle_to_turn = 180+a[0][0]
                    print("Angle to Rotate", 180+a[0][0])
                else:
                    angle_to_turn = (180-a[0][0])*(-1)
                    print("Angle to Rotate: ",(180-a[0][0])*(-1))

                print("shape of depth_frame", np.shape(depth_frame))
                #print("corners", corners[0][0]+corners[1][0]+corners[2][0]+corners[3][0])//4, (corners[0][1]+corners[1][1]+corners[2][1]+corners[3][1])//4))
#                print("corners",corners[i][0][0][0], i)
 #               print("corners", corners)
       

                depth_array.append(depth_frame[(int(corners[i][0][0][1])+int(corners[i][0][1][1])+int(corners[i][0][2][1])+int(corners[i][0][3][1]))//4,(int(corners[i][0][0][0])+int(corners[i][0][1][0])+int(corners[i][0][2][0])+int(corners[i][0][3][0]))//4])

                #depth_array = depth_frame[(x00+x01+x02+x03)//4, (x10+x21+x31+x41)//4]
            '''
            if depth<=1.5:
                rotate_angle = 90 - angle_to_turn
            '''
#        depth_array = np.nan_to_num(depth_array, copy=True, nan= 100000, posinf = 100000, neginf = 100000)
        return ret, frame, ids, depth_array

    def rotation_vector_to_euler_angles(self,rvec,tvec):
        # Convert rotation vector to rotation matrix
        rotation_mat, _ = cv2.Rodrigues(rvec)
        pose_mat = cv2.hconcat((rotation_mat, tvec))
        _, _, _, _, _, _, euler_angles = cv2.decomposeProjectionMatrix(pose_mat)
        return euler_angles
    

    def detectAruco(self):
        model = YOLO("./weights/aruco_main.pt")
#        self.color_image 
        depth_frame = self.depth_image
        ret = False
        print("img shape", np.shape(self.color_image))
        results = model.predict(self.color_image, conf=0.45, max_det = 2)
        depth_array = []
        for r in results:
            annotator = Annotator(self.color_image)
            boxes = r.boxes
            for box in boxes:
                ret  = True
                b = box.xyxy[0]  # get box coordinates in (left, top, right, bottom) format
                c = box.cls
                con = box.conf
                annotator.box_label(b, model.names[int(c)])

                # Get dep/home/kavin/caesar2020_nvidia/src/navigation/scripts/kavin_modifi_2.pyth data for the bounding box
                left, top, right, bottom = map(int, b)
                arrow_center = (left+right)/2
                try:
                    dist = depth_frame[(top+bottom) // 2, (left+right) // 2]
                    if dist != 0.0:
                        depth_array.append(dist)
                        #ret = True
                        print("Depth for box" + str(b) + ":" + str(depth_array[-1]) +"meters")
#                        self.start_time = time.time()   #this is for reducing the frequency of hitting search
                except:
                    ret = False
        if len(depth_array)>=2:
            depth_array = [depth_array[0], depth_array[1]] #min distance
        elif len(depth_array)==1:
            depth_array = [depth_array[0], 100000]
        else:
            depth_array = [100000, 100000]
        depth_array= np.nan_to_num(depth_array, copy = True, nan= 100000, posinf = 100000, neginf = 100000)
        #cv2.imshow("Image", self.color_image)      
       # key = cv2.waitKey(1) & 0xFF
        #if key == ord('q'):
         #   return
        return ret, depth_array

        
    
    def search(self):
        
        if abs(self.enc_data<4 and self.ret and not self.init):
            return
        
        print("Search function is active rn")
        if time.time() - self.start_time < 25 and self.init == False:
            print("search will occur once this time becomes 25:", time.time()-self.start_time)
            return
        
        msg1 = WheelRpm()   # (dummy) message just to make sure rover has stopped when search is happening
        msg1.hb = False
        msg1.omega = 0
        msg1.vel = 0
        wheelrpm_pub.publish(msg1)
        print("Rover has stopped")

        msg = std_msgs.Int32MultiArray()    #message type for the arm
        msg.data = [0,0,0,0,0,0]

        #Standard code
        msg.layout = std_msgs.MultiArrayLayout()
        msg.layout.data_offset = 0
        msg.layout.dim = [std_msgs.MultiArrayDimension()]
        msg.layout.dim[0].size = msg.layout.dim[0].stride = len(msg.data)
        msg.layout.dim[0].label = 'write'
        self.pub.publish(msg)

        print("Entered while loop.")
        #mini = time.time()
        while (self.init == False and abs(self.enc_data) < abs(self.start_angle)-2*self.angle_thresh):
            #to make the realsense go to the 60 degree maximum before starting the burst search

            msg.data = [0,0,100,0,0,0]
            print("camera is turning")
            rate.sleep()
            self.pub.publish(msg)
        self.start_time = time.time() - 4  
            #if time.time() - mini >4:
                #break
        msg.data = [0,0,0,0,0,0]
        print ("Exited while loop.")

        self.init = True
        print("Realsense's maximum angle:", self.enc_data)
        print("self.ret:", self.ret)

        if self.init == True and abs(self.enc_data) < (abs(self.start_angle)-0.5*self.angle_thresh) and self.ret == False:
            # if arrow is not detected and the realsense has not gone beyond the 60 degree maximum, continue moving realsense
            # self.init is not made false, so this will happen again when search is called in main()

            print("Camera Moving towards opp extreme")
            msg.data = [0,0,-100,0,0,0]
            rate.sleep()
            self.pub.publish(msg)
            print()

        elif self.init == True and abs(self.enc_data)<abs(self.start_angle) and self.ret == True:
            #if arrow is detected and the realsense is within the 60 degree maximum, append the arrow's values and continuemoving realsense
            # self.init is not made false, so this will happen again when search is called in main()
            print("is this for loop the problem?")
            print()
            for i in range(0,len(self.distance_list),1):    #what the heck is happening here and how has it not given a compilation error yet T-T  #Now it won't :)
                if self.distance_list[i] <15000 and self.distance_list[i] != 0.0:
                    self.angles_dict[self.distance_list[i]].append(self.enc_data)
                    print("I'm appending to angles_dict")
            msg.data = [0,0,-100,0,0,0]
            self.pub.publish(msg)
            rate.sleep()
            print("Aruco detected at: ", self.enc_data)
            print()

        elif self.ret == False or abs(self.enc_data)>abs(self.start_angle):
            # when the realsense has crossed the 60 degree maximum, realsense comes back to middle and the dictionary is processed
            # self.init is set to false (when next search() is called, realsense will first move to the 60 degree maximum)
            # and the counting of time is reset (that is, the next search will happen at least 20s after this block of code)
            #mini = time.time()
            print("hunch")
            while abs(self.enc_data) > 4:
                print("while loop 2")
                msg.data = [0,0,100,0,0,0]
                rate.sleep()
                self.pub.publish(msg)
                #if time.time()-mini>4:
                    #break
            print("last part of search")
            msg.data = [0,0,0,0,0,0]
            self.pub.publish(msg)
            self.init = False
            self.searchcalled = True
            self.distance = 10.0
            self.start_time = time.time()
        
    

    def process_dict(self):
        print("The dictionary with dist:[enc_angles] : ", self.angles_dict)
        self.searchcalled = False
        max_length_key = 0.0

        try:
            print("Entered try of process_dict")
            if min(self.angles_dict.keys()) != 0.0:
                list1 = []
                len_list = list(self.angles_dict.keys())
                angle_list = list(self.angles_dict.items())
                for i in range(0,len(angle_list),1):
                    list1.append(angle_list[i][1][0])
                sum1 = 0
                for i in range(0,len(angle_list), 1):
                    sum1 += list1[i]
                    if i >0:
                        if list1[i] == list1[i-1]:
                            sum1+=list1[i-1]
                self.angle_to_turn = sum1/len(list1)

                print('len_list',len_list)
                print('angle_list', angle_list)
                print("call for debug1")
                #self.angles_dict = OrderedDict(sorted
                print()
#                print("is the issue the for loop in process dict?")
        except Exception as e:
            print(e)
            print()
            print("The list is empty. No min value")
            self.init = False
            self.searchcalled = False
            return
       
        
        print("Angle to turn: ", self.angle_to_turn)
        print("the dictionary with dist[enc_angles]: " ,self.angles_dict)

   #     if self.which_enc_angle_to_turn < 0:
  #          self.direction = "left"
        self.rotate_angle = self.angle_to_turn

    #    else:
 #           self.direction = "right"
     #       self.rotate_angle = abs(self.which_enc_angle_to_turn)

        self.turn = True
#        self.initial_yaw = self.z_angle
        self.angles_dict = defaultdict(list)


    def main(self):
        if self.led_published == False:
            self.rover_state=Int32()
            self.rover_state=2 #initially its in yellow colour
            self.led_pub.publish(self.rover_state)
            self.led_published = True
        print()
        print("self.init in main", self.init)
        print("self.z_angle in main", self.z_angle)
        print("self.initial_yaw" , self.initial_yaw)
        print("self.initial_drift", self.initial_drift_angle)
        print()
        if (self.ret == False and self.turn == False) or self.init:
            self.search()
            if self.searchcalled:
                self.process_dict()
        if not self.turn:
            self.ret, self.distance_list = self.detectAruco() #incorporate no_arucos later
            distance_list=min(self.distance_list)/1000
            if distance_list < 4:
                self.distance = distance_list
                print(self.distance, "self.distance")
                print("distance list in main:",distance_list)
                if distance_list<4:
                    ret,ids, distance = self.aruco_recognition(self.color_image, self.depth_image) #distance is not proper
                    if ret == True:
                        self.ret = True
                    #else:
                        #self.ret = False
                if self.distance < 2.0 and self.distance >0.0 and self.ret == True:
                    print("Motion is being arrested")
                    self.arrest_motion = True
 
                if self.ret:
                    print(f"Aruco {ids} detected at distance: {self.distance}")
                    print("Direction:")
                    # else:
                    #     print("Trying to find aruco ....")
            self.move_straight()
        elif self.ret == False and self.turn == False:
            print("Trying to find aruco ....")
            self.move_straight()

        elif self.turn == True: # so self.turn = True in this case. 
            #This iteration's idea is to stop at the closest point and do search 
            print("I'm inside the ROTATE block")
            self.move_straight()


    def move_straight(self):
        msg = WheelRpm()
        msg.hb = False

        #drift correction
       # if (abs(self.initial_yaw-self.z_angle) > 10 and self.searchcalled == False and self.init == False):    #only if large drift (of 10 degrees) is there, correct it
            #print("self.initial_drift and self.z", self.initial_drift_angle, self.z_angle)
           # while (abs(self.initial_yaw-self.z_angle) > 6):    
                #while it doesn't come back to near the proper orientation, keep giving omega
               # kp_straight_rot= 1
               # msg.omega=int(0-kp_straight_rot*(self.initial_yaw-self.z_angle))
               # msg.omega+=10*int((self.initial_yaw-self.z_angle)/abs(self.initial_yaw-self.z_angle))   #in case it can't rotate fast    # c*x/abs(x) is basically c*(sign of x)
                
                #capping velocity
                #if(msg.omega>25):
                 #   msg.omega = 25
                #elif(msg.omega<-25):
                #    msg.omega = -25
               # msg.vel=0
               # print("correcting drift with omega =", msg.omega)
               # wheelrpm_pub.publish(msg)
               # rate.sleep()
           # rospy.sleep(1)
        
       # msg.omega=0

        if self.init or self.searchcalled:
            print("move_straight() is being ignored due to search().")
            msg.vel = 0
            wheelrpm_pub.publish(msg)
        
        elif self.turn:
            self.rotate()
        #elif abs(self.z_angle-self.initial_drift_angle) > 10:
            #self.rotate_angle = self.z_angle - self.initial_drift_angle
            #self.rotate()
        
        elif self.ret == True:
            print("sus area")
            if self.distance>2:
                msg.vel = 30 
                #wheelrpm_pub.publish(msg)
            #else:
                #print("Nearing the aruco")
                #if self.where_am_i == 0:
                    #msg.vel == 20
                    #wheelrpm_pub.publish(msg)
                #THE GREY REGION
                #if self.where_am_i == 1 and self.dire_attempt == 1:
                    #msg.vel == -20
            else:
                self.arrest_motion = True
            wheelrpm_pub.publish(msg)
        elif self.arrest_motion == True:
             msg.vel =0
             msg.omega = 0
             self.code_stopping_variable = True
             self.check_msg.data=2
             self.execute_goal=False
             self.rover_state=1
             self.led_pub.publish(self.rover_state)
             self.check_publisher.publish(self.check_msg)
#             self.check_pubb
             wheelrpm_pub.publish(msg)

        else:
            msg.vel = 30
            msg.omega = 0
            wheelrpm_pub.publish(msg)
        print(msg)

    def spin(self):
        while not rospy.is_shutdown():
            print()
            print("self.code_stopping_variable", self.code_stopping_variable, "self.execute_goal", self.execute_goal)
            if self.code_stopping_variable == False and self.execute_goal==True:
                self.main()
                #self.check_publisher.publish(self.check_msg)
                self.counter == 1

            #elif self.state == False:
                #print("Rover in Joystick mode")
                #self.initial_drift_angle=self.z_angle   #to prevent the drift correction from thinking that the manual control of the rover is a drift
            elif self.counter == 1:
                print("Im inside spin's task completion")
                self.check_publisher.publish(self.check_msg)
                self.task_completed_pub.publish(1)
                self.counter = self.counter + 1
            else:
                print("Waiting for check")
            rate.sleep()  

    def state_callback(self,msg):
        self.state = True

    def yaw_callback(self,data):
        current_x = data.orientation.x
        current_y = data.orientation.y
        current_z = data.orientation.z
        current_w = data.orientation.w
        current_tuple=(current_x, current_y, current_z, current_w)
        current_pitch, current_roll, self.z_angle = euler_from_quaternion(current_tuple)
        if self.initial_yaw == 0:
            self.initial_yaw = self.z_angle*180/math.pi
        self.z_angle = self.z_angle*180/math.pi - self.initial_yaw
        #print("self.z_angle in callback", self.z_angle)

        if self.z_angle < -179:
            self.z_angle = self.z_angle + 360
        elif self.z_angle > 179:
            self.z_angle = self.z_angle - 360


    def enc_callback(self,msg):
        self.enc_data = msg.data[4]

    def gps_callback(self,msg):
        if(msg.latitude and  msg.longitude):
            self.current_latitude=msg.latitude
            self.current_longitude=msg.longitude
            file_object=open("coordinates.txt","a")
            file_object.write(f"latitude :{msg.latitude}, longitude :{msg.longitude} \n")
            file_object.close()

    def rotate(self):
        print("Rotation has been called")
        msg = WheelRpm()
        msg.vel = 0
        msg.hb=False
#        if (diff > 120):
 #           diff = diff - 360
  #      elif (diff < -120):
   #         diff = diff + 360
    #    print("diff=",diff)        
     #   if diff > 0:
      #      dir = -1
       # else:
       #     dir = 1
 
        '''
        if (-60<self.z_angle-self.initial_yaw<60):
            error=30
        else:
            error = 90-abs(self.z_angle-self.initial_yaw)
        '''
        print("Rotation angle:",self.rotate_angle)
        print("Initial Yaw:", self.initial_yaw)
        print("Current z angle", self.z_angle)
#        error1 = self.z_angle  
        if self.rotate_start == False:
            self.temp_z = self.z_angle
            self.rotate_start = True
        error = self.z_angle - self.temp_z
        print("error=", error)
        #if self.direction == -1:
         #   self.rotate_angle = self.rotate_angle +2
        if (self.rotate_angle)>0:
            dirn = 1
        else:
            dirn = -1
        print()
        print("self.temp_z and self.z_angle", self.temp_z, self.z_angle)
        print()
        if (abs(error)<=(abs(self.rotate_angle)-2)):
            msg.omega=(dirn*30)
            msg.vel=0
            print("Calling Rotate, printing Z angle below")
            print(error)
            wheelrpm_pub.publish(msg)
        else:
            msg.omega=0
            msg.vel = 0
            wheelrpm_pub.publish(msg)
            # self.z_angle=0

            self.initial_drift_angle=self.z_angle
            print("****ROTATE DONE*****")
            #self.distance = 10.0
            self.start_time = time.time()-10
            self.turn=False
            self.direction = "Not Available"
            self.initial_yaw = self.z_angle
            self.rotate_start = False
            rospy.sleep(2)
    

    def stop_run(self):
        msg_stop=WheelRpm()
        msg_stop.hb=False
        msg_stop.vel=msg_stop.omega=0
        wheelrpm_pub.publish(msg_stop)
#        self.bag.close()


if __name__ == '__main__':
    try:
        rospy.init_node("aruco_detection_arc1")
        rate = rospy.Rate(10)
        wheelrpm_pub=rospy.Publisher('motion',WheelRpm,queue_size=10)
        gps_data_pub = rospy.Publisher('gps_bool',std_msgs.Int8,queue_size=10)
        crab_motion_pub = rospy.Publisher('crab_bool', std_msgs.Int8, queue_size=10)
        run = Aruco_detection()
        run.spin()
    except KeyboardInterrupt:
        sys.exit()

