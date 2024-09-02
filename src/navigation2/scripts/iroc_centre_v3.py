#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Int32, MultiArrayLayout, MultiArrayDimension, Bool
import numpy as np
import cv2
from cv_bridge import CvBridge
from navigation2.msg import red

class find_center():
    def __init__(self):
        #self.up=np.array([140,255,255])
        #self.down=np.array([80,100,30])
        self.down = np.array([110,100,15])
        self.up = np.array([127,255,255])
        rospy.Subscriber('/zed2i/zed_node/rgb/image_rect_color', Image, self.get_cframe)
        rospy.Subscriber('/zed2i/zed_node/depth/depth_registered', Image, self.get_dframe)
        self.velocity_pub = rospy.Publisher("/anuj_red", red, queue_size = 10)
        self.ik_pub = rospy.Publisher('/goal_blue', Int32, queue_size = 10)
        rospy.Subscriber('/ik_over_ah', Bool, self.ik_callback)
        self.rate = rospy.Rate(10)
        self.image_arrived = False
        self.depth_arrived= False
        self.contour_ended = False
        self.cframe = np.zeros((360,640))
        self.dframe = np.zeros((360,640))
        self.goal_pub = rospy.Publisher('arm_goal', Float32MultiArray, queue_size = 10)
        self.goal_dist = False
        self.goal_angle = False
        self.final_list_x = []
        self.final_list_y = []
        self.depth_1 = []
        self.depth_2 = []
        self.cord_x = self.cord_y = self.cord_z = 0
        self.worst_case_x = self.worst_case_y = 0
        self.pickup = False
        self.counter = 0
    
    def ik_callback(self, msg):
        pickup = False
        pickup = msg.data
        if pickup == True:
            self.pickup = True

    def get_dframe(self,data):
        bridge=CvBridge()
        dframe=data
        if self.contour_ended == True:
            self.dframe=bridge.imgmsg_to_cv2(dframe, "passthrough")
        #print(type(self.dframe))
        #if np.all(self.dframe)!= 0:
        self.depth_arrived =True
        #lse:
         #  self.depth_arrived = False
    def get_cframe(self,data):
        bridge=CvBridge()
        cframe=data
        if self.contour_ended == True:
            self.cframe=bridge.imgmsg_to_cv2(cframe, "bgr8")
        #print(type(self.cframe))
        #if np.all(self.cframe)!= 0:
        self.image_arrived = True
       #else:
            #self.image_arrived = False

    def get_centre(self):
        X = self.x + int(self.w/2)
        Y = self.y + 5
        print("Y", Y)
        Y_array = []
        D_array = []
        print("hi4")
        d1 = self.get_depth(self.x + self.w/2, Y)
        print("d1", d1)
        while Y < (self.y + self.h):
            Y_array.append(Y)
            d = self.get_depth(self.x + self.w/2, Y)
            print("d", d)
            if d == None or d1 == None:
                pass
            else:
                D_array.append(d1 - d)
            d1 = d
            Y = Y + 2
        D_array=D_array[:len(D_array)-1]
        print("D_array", D_array)
        Y_array = Y_array[:len(Y_array)-1]
        print("before declaration of selfcheck")
        self.check = []
        for i in range(len(D_array) - 1):
            if (D_array[i] >= 0 and D_array[i+1] <= 0) or (D_array[i+1] >= 0 and D_array[i] <= 0):
                self.check.append(i + 1)
        print("before self.p_x and self.p_y")
        print("self.check", self.check)
            
        if len(self.check) >= 3:
            self.p_x = int(self.x + self.w/2)
            self.p_y = int((Y_array[self.check[0]] + Y_array[self.check[1]])/2)
            if len(self.final_list_x) <=50 and len(self.final_list_y) <=50:
                self.final_list_x.append(self.p_x)
                self.final_list_y.append(self.p_y)
            else:
                self.final_list_x.pop(0)
                self.final_list_y.pop(0)
                #self.final_list_x.append(self.p_x)
                #self.final_list_y.append(self.p_y)

            print("##################")
            print(self.final_list_x)
            print("##################")
            #if len(self.final_list_x) >=20 and len(self.final_list_y) >= 20:
            self.p_x = int(sum(self.final_list_x)/len(self.final_list_x))
            self.p_y = int(sum(self.final_list_y)/len(self.final_list_y))
            cv2.circle(self.cframe, (self.p_x, self.p_y), 7, (0, 0, 0), -1)
            print('type check inside self.check', type(self.cframe))
            print("hi2")
            y1 = Y_array[self.check[0]]
            y2 = Y_array[self.check[1]]
           # y1 = self.y
            #y2 = self.worst_case_y
            #self.d1=self.get_depth(self.x+self.w/2, Y_array[self.check[0]])
            #self.d2=self.get_depth(self.x+self.w/2, Y_array[self.check[1]])
            self.d1=self.get_depth(self.x+self.w/2, y1)
            self.d2=self.get_depth(self.x+self.w/2, y2)
 
            #if not np.isnan(self.d1):
#            self.depth_1.append(self.d1)
 #           self.depth_2.append(self.d2)

            self.get_coords()
            msg = Float32MultiArray()
            msg.data = [0,0,0]
            msg.layout = MultiArrayLayout()
            msg.layout.data_offset = 0
            msg.layout.dim = [MultiArrayDimension()]
            msg.layout.dim[0].size = msg.layout.dim[0].stride = len(msg.data)
            msg.layout.dim[0].label = 'write'
            msg.data = [-self.cord_y-0.07,self.cord_z-0.10,self.cord_x - 0.07]
            #msg.data = [-self.cord_y - 0.17, 0.5, self.cord_x - 0.12]
            print("msg", msg)
            if self.cord_x != 0 and self.cord_y != 0 and self.cord_z != 0 and self.d <= 1.5 and not np.isnan(self.d):
                self.goal_pub.publish(msg)

    def contour(self):
        self.contour_ended = True
        print(f"Image Arrived = {self.image_arrived}, Depth Arrived = {self.depth_arrived}, Len = {np.shape(self.cframe)}, {np.shape(self.dframe)}")
        if self.image_arrived == True and self.depth_arrived == True and len(np.shape(self.cframe)) >= 3 and len(np.shape(self.dframe)) >= 2:
            #print("Aye boss ", len(np.shape(self.cframe)))
            blue_blur = cv2.GaussianBlur(self.cframe, (3,3), 0)
            #blue_blur = cv2.medianBlur(self.cframe, 7)
            #print("Aye boss (version cvt)", len(np.shape(self.cframe)))
            blue_hsv = cv2.cvtColor(blue_blur, cv2.COLOR_BGR2HSV)
            blue_mask = cv2.inRange(blue_hsv, self.down, self.up)
            contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            if len(contours) == 0:
                #cv2.imshow("noice", self.cframe)
                g = red()
                g.detect = False
                self.velocity_pub.publish(g)
                self.ik_pub.publish(0)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    return
            else:
                self.goal_dist = False
                self.goal_angle = False
                area = []
                Y_array = []
                self.D_array = []
                for i in contours:
                    area.append(cv2.contourArea(i))
                    #print("hi3")
                contour = contours[area.index(max(area))]
                app = cv2.approxPolyDP(contour, 0.005*cv2.arcLength(contour, True), True)
                self.x, self.y, self.w, self.h = cv2.boundingRect(app)
                if len(np.shape(self.cframe)) == 3:
                    cv2.rectangle(self.cframe, (self.x, self.y), (self.x + self.w, self.y + self.h), (255, 0, 0), 2, 0)
                #M = cv2.moments(contour)
                #self.cX = int(M['m10']/M['m00'])
                #self.cY = int(M['m01']/M['m00'])
                self.cX = self.x + int(self.w/2)
                self.cY = self.y + int(self.h/2)
                self.worst_case_x = int(self.x + self.w/2)
                self.worst_case_y = int(self.y + 1)
                depth_data=self.get_depth(self.cX, self.cY)
                g = red()
                if self.w>20 and self.h >20:
                    g.detect = True
                else:
                    g.detect = False
                kp = 26.67
                #if depth_data > 0.75:
                g.vel =35
                if depth_data > 0.5:
                    g.vel = int(min(kp * depth_data, 20))
                    #g.vel = 0
                elif (self.cY > 330 or depth_data < 0.5) and g.detect == True:
                    g.vel = 0
                    self.goal_dist = True
                    print("############################################################")
                    self.ik_pub.publish(-1)

                 #print("msg", msg)
                if self.cX > 360:
                    g.omega = 40
                elif self.cY <150 and self.cX>0:
                    g.omega = -40
                else:
                    g.omega = 0
                    self.goal_angle = True
                print("G is amazing", g)
                #msg.data = [0,0,0,0,0,0]
                self.velocity_pub.publish(g)

                #goal_pub.publish(msg)
                #print("publsihsing")
                print(f'Depth: {self.get_depth(self.cX, self.cY)}')

                if self.get_depth(self.cX, self.cY) > 2.0 and len(np.shape(self.cframe)) != 0:
                    print("not close enough bruh")
                    cv2.imshow("noice", self.cframe)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        return
                else:
                    self.get_centre()
                    cv2.imshow("noice", self.cframe)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        return
        self.contour_ended = True

    def get_depth(self, x, y):
        #if x < 360 and y < 640:
        print("x and y in get_depth", x, y)
        depth = self.dframe[int(y), int(x)]
        return depth
        #else:
            #return 100

    def get_coords(self):
        f_x=527.2972398956961
        f_y=527.2972398956961
        c_x=658.8206787109375
        c_y=372.25787353515625
        #v1 = self.p_x
        #v2 = self.p_y 
        v1 = self.worst_case_x
        v2 = self.worst_case_y
        self.d=np.sqrt(0.5*(self.d1**2+self.d2**2-(0.14**2)/2))
        if self.d <= 1.5 and not np.isnan(self.d):
        #if not np.isnan(self.d) and len(self.depth_1) <= 50:
            #self.depth_1.append(self.d)
        #print("length of d array", self.depth_1)
        #if len(self.depth_1) != 0:
            #self.d = sum(self.depth_1)/len(self.depth_1)
            self.cord_x=self.d*(v1-c_x)/f_x
            self.cord_y=self.d*(v2-c_y)/f_y
            self.cord_z=self.d

    def spin(self):
        while not rospy.is_shutdown():
            if self.pickup == True and self.image_arrived == True and self.depth_arrived == True:
                self.contour()
            else:
                print("waiting for tanish to finish")
            rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('iroc_centre', anonymous=True)
        rate = rospy.Rate(5)
        auto = find_center()
        auto.spin()
    except rospy.ROSInterruptException:
        pass
