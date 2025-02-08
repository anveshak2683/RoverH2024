#!/usr/bin/env python3 
import numpy as np
import cv2
import rospy
import glob
import imutils
import argparse
import time
import os
import sys
from std_msgs.msg import MultiArrayLayout, MultiArrayDimension, Float32MultiArray
from sensor_msgs.msg import Joy
import subprocess
class Cam_Servo:
    timer_delta_t_in_seconds = 0.8
    speed_msg=Float32MultiArray()
    speed_msg.layout=MultiArrayLayout()
    speed_msg.layout.data_offset = 0
    speed_msg.layout.dim = [ MultiArrayDimension() ]
    speed_msg.layout.dim[0].label = 'write'
    def start_camera(self):
       self.cam=cv2.VideoCapture(self.cam_no)
       self.cam.open(self.cam_no)
       time.sleep(3)
    def __init__(self) : #this function is used for filtering out the black out images
       self.cam_no=int(input("enter the camera id"))
       self.usb_port=int(input("enter the id"))
       test = subprocess.Popen(["rosrun", "rosserial_python", "serial_node.py", "_baud:=115200", "/dev/ttyUSB"+str(self.usb_port)])
       self.pan_speed=0
       self.tilt_speed=0
       self.speed=[self.pan_speed,self.tilt_speed]
    st_flag=0
    pan_no=0
    images=[]
    cam_state=False
    first_state=True
    ret=True
    def capture_image(self): #this is used for 
        #print("g")
        if self.first_state:
           self.start_camera()
           self.pan_speed=0
           self.first_state=False
        self.ret,image =self.cam.read()
        if self.ret:
            file_name="unstichedImages/TLR"+str(self.st_flag)+".png"
            cv2.imwrite(file_name,image) 
            # saving image in local storage 
            self.images.append(image)
        else:
             self.st_flag=self.st_flag-1
             print(self.ret)
    
    def create_Panaroma(self):
        #images_paths=glob.glob("unstichedImages/*.png")
        print("length:",len(self.images))
        stitcher=cv2.Stitcher.create()
        stiched_img=stitcher.stitch(self.images)
        (self.pano_status,self.stitched)=stiched_img
        print(self.pano_status)
        if self.pano_status==0:
            #cv2.imshow("panaroma_image",self.stitched)
            #cv2.waitKey(500)
            print("image got stitched")
            cv2.imwrite("panaroma_image.png",self.stitched)
            self.crop()
        else:
            print("failure")
        self.first_state=True
        self.images=[]
        self.speed_msg.data=[0,0]
    def crop(self):
          self.stitched=cv2.copyMakeBorder(self.stitched,10,10,10,10,cv2.BORDER_CONSTANT,(0,0,0))
          bw=cv2.cvtColor(self.stitched,cv2.COLOR_BGR2GRAY)
          thresh=cv2.threshold(bw,0,255,cv2.THRESH_BINARY)[1]
          cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
          cnts = imutils.grab_contours(cnts)
          c=max(cnts,key=cv2.contourArea)
          mask=np.zeros(thresh.shape,dtype="uint8")
          (x,y,w,h)=cv2.boundingRect(c)
          cv2.rectangle(mask,(x,y),(x+w,y+h),255,-1)
          minrect=mask.copy()
          sub=mask.copy()
          while cv2.countNonZero(sub)>0:
               minrect=cv2.erode(minrect,None)
               sub=cv2.subtract(minrect,thresh)
          print("I came out")
          ncnts = cv2.findContours(minrect.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
          ncnts = imutils.grab_contours(ncnts)
          c = max(ncnts, key=cv2.contourArea)
          (x, y, w, h) = cv2.boundingRect(c)
          self.fin_pan=self.stitched[y:y+h,x:x+w]
          nw=h*3
          self.fin_pan=cv2.resize(self.fin_pan,(nw,h))
          #cv2.imshow("Finale",self.fin_pan)
          #cv2.waitKey(500)
          cv2.imwrite("Stichedoutput.png"+str(self.pan_no),self.fin_pan)
          self.pan_no+=1
          
    def velocity_calculate(self,data):
            if data.buttons[-1]==1:
             self.speed=[0,0]
             self.cam_state=True
             self.timer_delta_t_in_seconds = 1
            else:
             if data.axes[7]>0:
               self.tilt_speed-=15
             elif data.axes[7]<0:
                self.tilt_speed+=15
             elif data.axes[6]>0:
                self.pan_speed+=15
             elif data.axes[6]<0:
                self.pan_speed-=15
            self.tilt_speed=self.tilt_speed%91
            self.pan_speed=self.pan_speed%181
            self.speed=[self.pan_speed,self.tilt_speed]
            self.speed_msg.data=self.speed
            print("speed in callback:-",cam1.speed)
    def callback(self,data):
	      self.velocity_calculate(data)
    def timerCallback(self,event):
      if self.cam_state:
           print("i am inside this loop with img count as",cam1.st_flag)
           self.pan_speed=self.pan_speed+10
           self.capture_image()
           print("pan_speed is ",self.pan_speed)
           self.st_flag+=1 
           if self.st_flag>13:
              self.cam_state=False
              print("cam_state in first if is",cam1.cam_state)
      print("i am outside this loop with img count as",cam1.st_flag)
      if self.st_flag==14:
         self.cam.release()
         self.create_Panaroma()
         self.timer_delta_t_in_seconds = 0.8
         self.st_flag=0
      self.pan_speed=self.pan_speed%181
      print("hello pan_speed is ",self.pan_speed)
      self.speed=[self.pan_speed,self.tilt_speed]
      self.speed_msg.data=self.speed
      pub.publish(self.speed_msg)

if __name__=="__main__":
    try:
        print("hi")
        #test = subprocess.Popen(["rosrun", "rosserial_python", "serial_node.py", "_baud:=115200", "/dev/ttyUSB1"])
        rospy.init_node("Pan_tilt_control")
        cam1=Cam_Servo()
        rospy.Subscriber("/joy_arm",Joy,cam1.callback)
        pub=rospy.Publisher("/Servo_write",Float32MultiArray,queue_size=10)
        rate = rospy.Rate(10)
        rospy.Timer(rospy.Duration(cam1.timer_delta_t_in_seconds),cam1.timerCallback)
        rate.sleep()
        rospy.spin()
    except KeyboardInterrupt:
        print("BYE")
        sys.exit()
