#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import time
#cap=cv2.VideoCapture(2)



class Star_Detection():
    def __init__(self):
        self.upper_green = np.array([102, 255, 255])
        self.lower_green = np.array([83, 70, 105])
        self.lower_black=np.array([0,0,0])
        self.upper_black=np.array([180,255,50])
        rospy.Subscriber('/zed2i/zed_node/rgb/image_rect_color',Image, self.color_callback)
        rospy.Subscriber('/zed2i/zed_node/depth/depth_registered', Image, self.depth_callback)
        rospy.Subscriber('/robot/dlo/odom_node/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/zed2i/zed_node/imu', Imu, self.imu_callback)
        self.counter = 0
        self.final_list = []
        self.distance = 0
        self.reached_end = False                      # Will change it to true when the cardboard centre distance is being received.    
        self.distance = 0                   # distance of camera from the cardboard.# Using Realsense/Zed 2i ####
        self.color_came= False
        self.depth_came = False
        self.depth_color = 0.0
        self.star_detected = False
        self.image_arrived = False
        self.depth_arrived= False
        self.counter_final = 0
        self.x =[]
        self.y = []
        self.edge_threshold=5
        self.cframe=np.zeros((360,640))
        self.dframe=np.zeros((360,640))
        self.cropped_img=np.zeros((360,640))
        self.contour_ended = False
        self.initializer=1
        self.x_mid_g=0
        self.y_mid_g=0
        self.x_mid=0
        self.y_mid=0
        self.photo=np.zeros((360,640))
        self.current_pose_x = self.current_pose_y = 0
        self.theta = 0
        self.counter = 0
        self.start_time = time.time()

    
    def odom_callback(self, msg):
        self.current_pose_x = msg.pose.pose.position.x
        self.current_pose_y = msg.pose.pose.position.y
#            self.initial_pose_x = self.current_pose_x
 #           self.initial_pose_y = self.current_pose_y
    
    def imu_callback(self, msg):
        _,_,self.theta = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w] )
    
    def color_callback(self, data):
        try:
             bridge = CvBridge()
             if self.contour_ended == True:
                    self.cframe = bridge.imgmsg_to_cv2(data, 'bgr8')
                    self.color_came = True
                    self.image_arrived=True
            # else:
             #       self.color_came = True
        except CvBridgeError as e:
            print(e)

    def depth_callback(self, data):
        try:
             bridge = CvBridge()
             dframe=data
             if self.contour_ended == True:
                   self.dframe = bridge.imgmsg_to_cv2(data,'passthrough')
                   self.depth_came = True
                   self.depth_arrived=True
            # else:
             #      self.depth_came = True
                  # print("shape", np.shape(self.depth_image))
        except CvBridgeError as e:
            print(e)

    def is_star(self, contour):
        epsilon = 0.04 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        return len(approx) > self.edge_threshold
    
    def check_point_inside_contour(self, contour, point):
        return cv2.pointPolygonTest(contour, point, False) >= 0
        
    def vector_add(self,x,y,theta,d):
        x1=x+d*math.cos(theta)
        y1=y+d*math.sin(theta)
        return (x1,y1)
    
    #Detecting colour panel
    def color_detection(self): 
        self.contour_ended = False
        print("shape of depth image ", np.shape(self.dframe))
        print("shape of color image", np.shape(self.cframe))
        if self.image_arrived == True and self.depth_arrived == True:
            gray = cv2.cvtColor(self.cframe, cv2.COLOR_BGR2GRAY)
            hsv = cv2.cvtColor(self.cframe, cv2.COLOR_BGR2HSV)
            black_mask = cv2.inRange(hsv, self.lower_black, self.upper_black)
            green_mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        
            black_contours, _ = cv2.findContours(black_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            green_contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
            black_star_detect = False
            print(self.cframe)
        
            for contour in green_contours:
                if self.is_star(contour):
                    #cv2.drawContours(self.cframe, [contour], -1, (0, 255, 0), 2)
                    area = cv2.contourArea(contour)
                    if area > 200:
                        black_star_detect = True

                        x, y, w, h = cv2.boundingRect(contour)
                        self.x_mid = int(x + (w / 2))
                        self.y_mid = int(y + (h / 2))
                        font = cv2.FONT_HERSHEY_DUPLEX

                        for black in black_contours:
                            xg, yg, wg, hg = cv2.boundingRect(black)
                            self.x_mid_g = int(xg + (wg / 2))
                            self.y_mid_g = int(yg + (hg / 2))
                            point_inside = self.check_point_inside_contour(contour, (self.x_mid_g,self.y_mid_g))

                            if point_inside and black_star_detect:
                                #cv2.putText(self.cframe, "star", (x_mid - 20, y_mid), font, 1, (0, 0, 255), 4)
                                 self.star_detected = True
                                # _,img=cap.read()
                                 #print("image is printing--------",img)
                                 self.cropped_img=self.cframe[y:y+h,x:x+w]

        
            #self.star_detected = True

            if self.x_mid != 0 and self.y_mid != 0 :
                self.depth_color = self.dframe[self.y_mid,self.x_mid]
                print("##########################################################33")
                print("star detected")
                print("depth of star is",self.depth_color)
                (star_x,star_y)=self.vector_add(self.current_pose_x,self.current_pose_y,self.theta,self.depth_color)
                if self.depth_color<2 and self.counter < 5:
                    print("************************************cropped_img********")
                    print(np.shape(self.cropped_img))
                    gf = "image"+"_x_"+str(star_x)+"_y_"+str(star_y)+".png"
                    cv2.imwrite(gf,self.cropped_img)
                    self.counter = self.counter+1
                    if self.counter >=4:
                        self.start_time = time.time()
                if time.time() - self.start_time>120 and self.counter >=4:
                    self.counter = 0
            
                self.x_mid=0
                self.y_mid=0
            else:
                self.color_detected= False
                print("Nothing detected")
            #cv2.imshow("FRAME",self.cframe)

            #if cv2.waitKey(1)&0xFF==27:
              # cv2.destroyAllWindows()


        self.contour_ended = True


    def spin(self):
        while not rospy.is_shutdown():
             #if self.color_came==True and self.depth_came==True:
                 self.color_detection()
                 rate.sleep()
            # elif self.initializer==1:
             #    self.color_detection()
              #   rate.sleep()

        else:
            print("Waiting ")

if __name__=="__main__":
    rospy.init_node("star", anonymous=True)
    rate = rospy.Rate(10)
    ahh = Star_Detection()
    ahh.spin()
