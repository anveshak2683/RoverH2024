#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
import numpy as np
import cv2
import imutils
import tf
from cv_bridge import CvBridge
from std_msgs.msg import MultiArrayDimension, MultiArrayLayout, Float32MultiArray
from tf import LookupException,ConnectivityException,ExtrapolationException
from traversal.msg import WheelRpm
class find_center():
    def __init__(self):
        self.up=np.array([140,255,255])
        self.down=np.array([105,45,45])
        rospy.Subscriber('/zed2i/zed_node/rgb/image_rect_color', Image, self.get_cframe)
        rospy.Subscriber('/zed2i/zed_node/depth/depth_registered', Image, self.get_dframe)
        rospy.Subscriber("/ik_start_drop",Bool,self.ik_start_drop_callback)
        self.rate = rospy.Rate(10)
        self.ik_start_drop=False
    def ik_start_drop_callback(self,data):
        self.ik_start_drop=data
        print("self.ik_start_drop",self.ik_start_drop)
    def get_transform(self,msg):
        self.t=msg.transform.translation
        print(self.t)
    def get_dframe(self,data):
        bridge=CvBridge()
        self.dframe=data
        self.dframe=bridge.imgmsg_to_cv2(self.dframe, "passthrough")
    def get_cframe(self,data):
        bridge=CvBridge()
        self.cframe=data
        self.cframe=bridge.imgmsg_to_cv2(self.cframe, "bgr8")
        self.contour()
    def contour(self):
        blue_hsv=cv2.cvtColor(self.cframe,cv2.COLOR_BGR2HSV)
        mask=cv2.inRange(blue_hsv,self.down,self.up)
        ker=np.ones((7,7),np.uint8)
        mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,ker)
        mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,ker)
        cont=cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cont=imutils.grab_contours(cont)
        area=[]
        Y_array=[]
        print("hi")
        self.D_array=[]
        for i in cont:
            area.append(cv2.contourArea(i))
        if area==[]:
            pass
        else:
            print("hi3")
            z=max(area)
            i=area.index(z)
            app=cv2.approxPolyDP(cont[i],0.005*cv2.arcLength(cont[i],True),True)
            self.x,self.y,self.w,self.h=cv2.boundingRect(app)
            i1=cv2.rectangle(self.cframe,(self.x,self.y),(self.x+self.w,self.y+self.h),(255,0,0),2,0)
            Y=self.y+1
            print("hi4")
            d1=self.get_depth(self.x+self.w/2,Y-1)
            while Y<=self.y+self.h:
                Y_array.append(Y)
                d=self.get_depth(self.x+self.w/2,Y)
                self.D_array.append(d1-d)
                d1=d
                Y=Y+1
            self.D_array=self.D_array[:len(self.D_array)-1]
            print("self.D_array", self.D_array)
            
            Y_array=Y_array[:len(Y_array)-1]
            print("before declaration of self.check")
            self.check=[]
            for i in range(len(self.D_array)):
                if i==len(self.D_array)-1:
                    break
                else:
                    if (self.D_array[i]>=0 and self.D_array[i+1]<=0) or (self.D_array[i+1]>=0 and self.D_array[i]<=0):
                        self.check.append(i+1)
            print("before self.p_x and self.p_y")
            print("self.check", self.check)
            self.p_x=int(self.x+self.w/2)
            self.p_y=int((Y_array[self.check[0]]+Y_array[self.check[1]])/2)
            cv2.circle(self.cframe,(self.p_x,self.p_y),7,(0,0,0),-1)
            print("hi2")
            self.d1=self.get_depth(self.x+self.w/2,Y_array[self.check[0]])
            self.d2=self.get_depth(self.x+self.w/2,Y_array[self.check[1]])
            self.get_coords()
            #x, y, z = self.collectiontube_frame(self.cord_x,self.cord_y,self.cord_z)
            msg= Float32MultiArray()
            msg.data=[0,0,0]
            msg.layout = MultiArrayLayout()
            msg.layout.data_offset = 0
            msg.layout.dim = [MultiArrayDimension()]
            msg.layout.dim[0].size = msg.layout.dim[0].stride = len(msg.data)
            msg.layout.dim[0].label = 'write'
            #msg.data = [x,z-0.11,-y-0.36]  
            #msg.data = [-self.cord_y-0.24,self.cord_x,self.cord_z]
            msg.data = [-self.cord_y-0.24,self.cord_z,self.cord_x]
            g = WheelRpm()
            kp = 26.67
            if msg.data[1] > 0.75:
                g.vel =20
            elif msg.data[1] > 0.5 and msg.data[1] < 0.75:
                g.vel = int(min(kp * d, 20))
                    #g.vel = 0
            else:
                g.vel = 0
                    
            print("msg", msg)        
            if self.p_x > 500:
                g.omega = 50
            elif self.p_x <200 and self.p_x>0:
                g.omega = -50
            else:
                g.omega = 0    
            print("G is amazing", g)
            #msg.data = [0,0,0,0,0,0]
            velocity_pub.publish(g)

            goal_pub.publish(msg)          
            print("publsihsing")

            if cv2.waitKey(1) & 0xFF == ord('q'):
                return
    def get_depth(self,x,y):
        depth=self.dframe[int(y),int(x)]
        return depth
    def get_coords(self):
        f_x=527.2972398956961
        f_y=527.2972398956961
        c_x=658.8206787109375
        c_y=372.25787353515625
        self.d=np.sqrt(0.5*(self.d1**2+self.d2**2-(0.15**2)/2))
        self.cord_x=self.d*(self.p_x-c_x)/f_x
        self.cord_y=self.d*(self.p_y-c_y)/f_y
        self.cord_z=self.d
    def spin(self):
        self.main()
        self.rate.sleep()
    def main(self):
        while not rospy.is_shutdown():
            rospy.spin()
if __name__ == '__main__':
    try:
        rospy.init_node('iroc_centre', anonymous=True)
        rate = rospy.Rate(5)
        goal_pub = rospy.Publisher("arm_goal", Float32MultiArray, queue_size=10)
        velocity_pub = rospy.Publisher("motion", WheelRpm, queue_size = 10)
        auto = find_center()
        auto.spin()
    except rospy.ROSInterruptException:
        pass
