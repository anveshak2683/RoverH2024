#!/usr/bin/env python3
import sys
import rospy
#import rosbag
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
from ultralytics import YOLO
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics.utils.plotting import Annotator
from collections import defaultdict
from collections import OrderedDict
from cv2 import aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image as Image2



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

        self.distance_list = []
        self.initial_yaw = 0.0
        self.where_am_i = None
        self.enc_data = 0
        self.start_angle = 60
        self.start_angle_1 = 90
        self.start_time = time.time()
        self.angles_dict = defaultdict(list)
        self.tmp_dict = defaultdict(list)
        self.angle_thresh = 5
        self.z_angle = 0
        self.color_image = np.zeros((360,640))
        self.depth_image = np.zeros((360,640))
        self.initial_drift_angle = 0

        #variables used for controlling something

        self.ret = False
        self.init = False  #actually true, but for testing changed to false
        self.searchcalled = False #variable to control when and if to enter process_dict()

        self.turn = False
        self.state = True
        print("self.state", self.state)

        self.pub = rospy.Publisher('stm_write', std_msgs.Int32MultiArray, queue_size=10)

        try:
            rospy.Subscriber('state', Bool, self.state_callback)
            print("1")
            rospy.Subscriber('chatter',std_msgs.Float32,self.yaw_callback)
            print("2")
            rospy.Subscriber('enc_auto',std_msgs.Int8,self.enc_callback)
            print("3")  
            rospy.Subscriber('gps_coordinates', gps_data, self.gps_callback)
            print("4")
            rospy.Subscriber('/zed2i/zed_node/rgb/image_rect_color', Image, self.color_callback)
            print('5')
            rospy.Subscriber('/zed2i/zed_node/depth/depth_registered', Image, self.depth_callback)
            print('6')
        except KeyboardInterrupt:
            # quit
            sys.exit()

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
        
        img = color_image
        print(img.shape)
        ret, output, ids, depth_array = self.pose_estimation(img, depth_image, intrinsic_camera, distortion)        #returns a list of two entities at all times
        depth_array.sort()
        depth_array.remove(0.0)
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
            ret = True
            for i in range(len(ids)):
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

                
                depth_array = depth_frame[(corners[0][0]+corners[1][0]+corners[2][0]+corners[3][0])/4,(corners[0][1]+corners[1][1]+corners[2][1]+corners[3][1])/4]
                
            '''
            if depth<=1.5:
                rotate_angle = 90 - angle_to_turn
            '''
        return ret, frame, ids, depth_array

    def rotation_vector_to_euler_angles(rvec,tvec):
        # Convert rotation vector to rotation matrix
        rotation_mat, _ = cv2.Rodrigues(rvec)
        pose_mat = cv2.hconcat((rotation_mat, tvec))
        _, _, _, _, _, _, euler_angles = cv2.decomposeProjectionMatrix(pose_mat)
        return euler_angles
    

    def detectAruco(self):
        model = YOLO("./weights/best.pt")
#        self.color_image 
        depth_frame = self.depth_image
        img = self.color_image 
        ret = False
        print("img shape", np.shape(self.color_image))
        results = model.predict(img, conf = 0.4, max_det = 3)
        depth_array = []
        for r in results:
            annotator = Annotator(img)
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
                    dist = depth_frame[(left + right) // 2, (top + bottom) // 2]
                    if dist != 0.0:
                        depth_array.append(int(dist))
                        print("Depth for box" + str(b) + ":" + str(depth_array[-1]) +"meters")
                except:
                    ret = False
                
        if len(depth_array)>=2:
            depth_array = [depth_array[0], depth_array[1]] #min distance
        elif len(depth_array)==1:
            depth_array = [depth_array[0], None]
        else:
            depth_array = [None, None]
        return ret, depth_array
        
    
    def search(self):
        
        if abs(self.enc_data<4 and self.ret and not self.init):
            return
        
        print("Search function is active rn")
        if time.time() - self.start_time < 25:
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
        mini = time.time()
        while self.init == False and abs(self.enc_data) < abs(self.start_angle)-2*self.angle_thresh:
            #to make the realsense go to the 60 degree maximum before starting the burst search

            msg.data = [0,255,0,0,0,0]
            rate.sleep()
            self.pub.publish(msg)
            self.start_time = time.time() - 4  
            if time.time() - mini >4:
                break
        msg.data = [0,0,0,0,0,0]
        print ("Exited while loop.")

        self.init = True
        print("Realsense's maximum angle:", self.enc_data)
        print("self.ret:", self.ret)

        if self.init == True and abs(self.enc_data) < (abs(self.start_angle)-0.5*self.angle_thresh) and self.ret == False:
            # if arrow is not detected and the realsense has not gone beyond the 60 degree maximum, continue moving realsense
            # self.init is not made false, so this will happen again when search is called in main()

            print("Camera Moving")
            msg.data = [0,-255,0,0,0,0]
            rate.sleep()
            self.pub.publish(msg)
            print()

        elif self.init == True and abs(self.enc_data)<abs(self.start_angle) and self.ret == True:
            #if arrow is detected and the realsense is within the 60 degree maximum, append the arrow's values and continuemoving realsense
            # self.init is not made false, so this will happen again when search is called in main()

            for i in range(0,len(self.distance_list),1):    #what the heck is happening here and how has it not given a compilation error yet T-T  #Now it won't :)
                self.angles_dict[self.distance_list[i]].append(self.enc_data)
                print("I'm appending to angles_dict")
            msg.data = [0,-255,0,0,0,0]
            self.pub.publish(msg)
            rate.sleep()
            print("Aruco detected at: ", self.enc_data)
            print()

        elif self.ret == False:
            # when the realsense has crossed the 60 degree maximum, realsense comes back to middle and the dictionary is processed
            # self.init is set to false (when next search() is called, realsense will first move to the 60 degree maximum)
            # and the counting of time is reset (that is, the next search will happen at least 20s after this block of code)
            mini = time.time()
            while abs(self.enc_data) > 4:
                msg.data = [0,255,0,0,0,0]
                rate.sleep()
                self.pub.publish(msg)
                if time.time()-mini>4:
                    self.init == False
                    break

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
            if min(self.angles_dict.keys()) != 0:
                self.angles_dict = OrderedDict(sorted)
                self.list1 = []
                self.list2 = []
                list1_bool = True
                for i in (0,len(self.angles_dict),1):
                    if i == 0:
                        self.list1.append(sum(self.angles_dict[i])/len(self.angles_dict[i]))
                    d = self.angles_dict[i+1] - self.angles_dict[i]
                    if d < 2 and list1_bool == True:
                        self.list1.append(sum(self.angles_dict[i])/len(self.angles_dict[i]))
                    if d > 2:
                        list1_bool = False
                    if d < 2 and list1_bool == False:
                        self.list2.append(sum(self.angles_dict[i])/len(self.angles_dict[i]))
        except:
            print("The list is empty. No min value")
            self.init = False
            self.searchcalled = False
            return
        if len(self.list1)!=0 and len(self.list2)!=0:
                self.which_enc_angle_to_turn = (sum(self.list1)/len(self.list1) + sum(self.list2)/len(self.list2))/2
                self.where_am_i = 0
                self.tmp_dict = self.angles_dict
        elif len(self.list1)!=0 and len(self.list2)==0:
            self.which_enc_angle_to_turn = sum(self.list1)/len(self.list1)
            self.where_am_i = 1
        
        
        print("Angle to turn: ", self.which_enc_angle_to_turn)
        print("the dictionary with dist[enc_angles]: " ,self.angles_dict)

        if self.which_enc_angle_to_turn < 0:
            self.direction = "left"
            self.rotate_angle = abs(self.which_enc_angle_to_turn)

        else:
            self.direction = "right"
            self.rotate_angle = abs(self.which_enc_angle_to_turn)

        self.turn = True
        self.initial_yaw = self.z_angle
        self.angles_dict = defaultdict(list)


    def main(self):
        crab_motion_pub.publish(0)
        if (self.ret == False and self.turn == False) or self.init:
            #self.search()
            if self.searchcalled:
                self.process_dict()
        if not self.turn:
            self.ret, distance_list = self.detectAruco() #incorporate no_arucos later
            if distance_list[0] != None and distance_list[1] != None:
                self.distance = distance_list[0]
                if min(distance_list)<4:
                    ret,ids, distance = self.aruco_recognition(self.color_image, self.depth_image) #distance is not proper
                    if ret == True:
                        self.ret = True
                    if self.ret:
                        print(f"Aruco {ids} detected at distance: {self.distance}")
                        print("Direction:")
                    # else:
                    #     print("Trying to find aruco ....")
                    #     self.move_straight()
            else:
                print("Trying to find aruco ....")
                print("self.state", self.state)
                self.move_straight()

        else: # so self.turn = True in this case. 
            #This iteration's idea is to stop at the closest point and do search 
            print("I'm inside the ROTATE block")
            self.search()
            if self.searchcalled:
                self.process_dict()
            if not self.turn:
                self.ret, distance_list = self.detectAruco() #incorporate no_arucos later
                # self.distance_list = distance_list
                # self.distance_list.remove(0.0)
                # self.distance_list.remove(None)
                # self.distance = min(self.distance_list)
                if self.distance<4:
                    ret, pix, distance = self.aruco_recognition(self.color_image, self.depth_image)
                    if ret == True:
                        self.ret = True
                        self.distance = distance

                if self.where_am_i == 0:
                    if self.direction == "left":
                        self.rotate(1)
                    elif self.direction == "right":
                        self.rotate(-1)   
                elif self.where_am_i == 1:
                    #now inspect the temporary memory space
                    self.dire_attempt = 1
                    list1 = []
                    list2 = []
                    list1_bool = []
                    print(self.tmp_dict)
                    for i in (0,len(self.tmp_dict),1):
                        if i == 0:
                            list1.append(sum(self.tmp_dict[i])/len(self.tmp_dict[i]))
                        d = self.tmp_dict[i+1] - self.tmp_dict[i]
                        if d < 2 and list1_bool == True:
                            list1.append(sum(self.tmp_dict[i])/len(self.tmp_dict[i]))
                        if d > 2:
                            list1_bool = False
                        if d < 2 and list1_bool == False:
                            list2.append(sum(self.tmp_dict[i])/len(self.tmp_dict[i]))
                        avg1_tmp = sum(list1)/len(list1)
                        avg2_tmp = sum(list2)/len(list2)
                        avg1_ang = sum(self.list1)/len(self.list1)
                        if avg1_tmp > avg1_ang:
                            #move left in crab motion
                            crab_motion_pub.publish(1)
                        if avg1_tmp < avg1_ang:
                            #move right in crab motion
                            crab_motion_pub.publish(-1)
                        start_time = time.time()
                        while time.time()-start_time < 5:
                            wheelrpm_pub.publish(20)
                        wheelrpm_pub.publish(0)
                        crab_motion_pub.publish(0)
                        self.move_straight()

    def move_straight(self):
        msg = WheelRpm()
        msg.hb = False

        #drift correction
        if (abs(self.initial_drift_angle-self.z_angle) > 10 and self.searchcalled == False):    #only if large drift (of 10 degrees) is there, correct it.
            while (abs(self.initial_drift_angle-self.z_angle) > 6):    
                #while it doesn't come back to near the proper orientation, keep giving omega

                msg.omega=int(0+self.kp_straight_rot*(self.initial_drift_angle-self.z_angle))
                msg.omega+=10*int((self.initial_drift_angle-self.z_angle)/abs(self.initial_drift_angle-self.z_angle))   #in case it can't rotate fast    # c*x/abs(x) is basically c*(sign of x)
                
                #capping velocity
                if(msg.omega>35):
                    msg.omega = 35
                elif(msg.omega<-35):
                    msg.omega = -35
                msg.vel=0
                print("correcting drift with omega =", msg.omega)
                wheelrpm_pub.publish(msg)
                rate.sleep()
            rospy.sleep(1)
        
        msg.omega=0

        if self.init or self.searchcalled:
            print("move_straight() is being ignored due to search().")
            print()
            print("self.init",self.init)
            print("self.searchcalled", self.searchcalled)
            msg.vel = 0
            wheelrpm_pub.publish(msg)
        
        elif self.ret:
            if self.distance>2:
                msg.vel = 20 
                wheelrpm_pub.publish(msg)
            else:
                print("Nearing the aruco")
                if self.where_am_i == 0:
                    msg.vel == 20
                    wheelrpm_pub.publish(msg)
                #THE GREY REGION
                if self.where_am_i == 1 and self.dire_attempt == 1:
                    msg.vel == -20         


    def spin(self):
        while not rospy.is_shutdown():
            if(self.state==True):
                self.main()
                rate.sleep()

            else:
                print("Rover in Joystick mode")
                self.initial_drift_angle=self.z_angle   #to prevent the drift correction from thinking that the manual control of the rover is a drift
                rate.sleep()
  

    def state_callback(self,msg):
#        self.state = msg.data
        self.state = True

    def yaw_callback(self,msg):
        if (self.initial_drift_angle==0):   #only for 1st time
            self.initial_drift_angle=self.z_angle
        self.z_angle = msg.data

    def enc_callback(self,msg):
        self.enc_data = msg.data

    def gps_callback(self,msg):
        if(msg.latitude and  msg.longitude):
            self.current_latitude=msg.latitude
            self.current_longitude=msg.longitude
            file_object=open("coordinates.txt","a")
            file_object.write(f"latitude :{msg.latitude}, longitude :{msg.longitude} \n")
            file_object.close()

    def rotate(self,dir):
        print("Rotation has been called")
        msg = WheelRpm()
        msg.vel = 0
        msg.hb=False
        diff  = self.z_angle - self.initial_yaw
        if (diff > 120):
            diff = diff - 360
        elif (diff < -120):
            diff = diff + 360
        print("diff=",diff)
        '''
        if (-60<self.z_angle-self.initial_yaw<60):
            error=30
        else:
            error = 90-abs(self.z_angle-self.initial_yaw)
        '''
        print("Rotation angle:",self.rotate_angle)
        print("Initial Yaw:", self.initial_yaw)
        print("Current z angle", self.z_angle)
        error = self.rotate_angle-abs(diff)
        print("error=", error)
        #if self.direction == -1:
         #   self.rotate_angle = self.rotate_angle +2
        if (abs(error)>=0.5*self.angle_thresh):
            msg.omega=0+(dir*40)
            msg.vel=20
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
            self.distance = 10.0
            self.start_time = time.time()-10
            self.turn=False
            self.direction = "Not Available"
            rospy.sleep(2)
    

    def stop_run(self):
        msg_stop=WheelRpm()
        msg_stop.hb=False
        msg_stop.vel=msg_stop.omega=0
        wheelrpm_pub.publish(msg_stop)
#        self.bag.close()


if __name__ == '__main__':
    try:
        rospy.init_node("aruco_detection_arc")
        rate = rospy.Rate(10)
        wheelrpm_pub=rospy.Publisher('motion1',WheelRpm,queue_size=10)
        gps_data_pub = rospy.Publisher('gps_bool',std_msgs.Int8,queue_size=10)
        crab_motion_pub = rospy.Publisher('crab_bool', std_msgs.Int8, queue_size=10)
        run = Aruco_detection()
        run.spin()
    except KeyboardInterrupt:
        sys.exit()

