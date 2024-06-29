import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray,MultiArrayLayout,MultiArrayDimension
from std_msgs.msg import Int32MultiArray
import sys
import math

#theta1=shouolder pan-base
#theta2 :shoulder lift
#theta3 :elbow-position

class IK_IROC():
    def __init__(self):
#        self.enc_data = [0,0,0]
#        self.d=[0,0,0,0] #joint lengths
        self.a=[0,0.40,0.35,0] #link lengths
        self.px,self.py,self.pz = 0.75,0,0 #substitute values of position of object which we will get from detection part
        # rospy.init_node("arm_auto", anonymous=True)
        self.rate=rospy.Rate(10)
        self.motor_data=rospy.Publisher('/auto_arm_signals',Int32MultiArray,queue_size=10) 
        self.enc_data_sub=rospy.Subscriber('/enc_drive',Float32MultiArray,self.enc_callback)#the topic
        print("hello")
        self.enc_data = [0,0,0]
        self.init=False

    def angle_value(self):
        m=self.a[1]
        n=self.a[2]

        k=math.pow(self.px,2)+math.pow(self.py,2)+math.pow(self.pz,2)
        l=2*k-math.pow(m,2)
        if self.px == 0 and self.py != 0:
            theta1 = math.pi/2
        else:
            theta1=float(np.arctan(self.py/self.px))
        theta3=-float(np.arccos(m+math.pow(l,0.5)/(2*n-math.pi)))
        if theta3 > math.pi/2:
            theta3 = math.pi/2 - theta3
        elif theta3 < -math.pi/2:
            theta3 = -math.pi/2 - theta3
        theta2=float((np.arccos(self.pz/(2*k)))+(np.arccos(m+math.pow(l,0.5)/(4*k))))
        if theta2 > math.pi/2:
            theta2 = math.pi/2 - theta2
        elif theta2 < -math.pi/2:
            theta2 = -math.pi/2 - theta2
        #theta1 = math.pi/4
        #theta2 = math.pi/4
        #theta3 = math.pi/4
        #:print((np.arccos(self.pz/(2*k)))+(np.arccos((m+math.pow(l,0.5))/(4*k))))

       # print(np.arccos(self.pz/(2*k)))
        print(m+math.pow(l,0.5)/(4*k))
        print(f"theta1 = {theta1},theta2 = {theta2}, theta3 = {theta3}")  
        base = (180*theta1)/math.pi
        shoulder = (180*theta2)/math.pi
        elbow = (180*theta3)/math.pi

        msg=Int32MultiArray()
        msg.data=[0,0,0,0,0,0]

        msg.layout = MultiArrayLayout()
        msg.layout.data_offset = 0
#
        msg.layout.dim = [MultiArrayDimension()]
        msg.layout.dim[0].size = msg.layout.dim[0].stride = len(msg.data)
        msg.layout.dim[0].label = 'write'
  	
        msg.data = [0,0,0,0,0,0]

        if self.init == False and self.enc_data[0]<base-2:
            msg.data[2] = 50
            print("base_angle:",self.enc_data[0])

        elif self.init == False and self.enc_data[0]>base+2:
            msg.data[2] = -50
            print("base_angle:",self.enc_data[0])


        elif self.init == False and self.enc_data[1]<shoulder-2:
            msg.data[4] = 50
            print("shoulder__angle:",self.enc_data[1])

        elif self.init == False and self.enc_data[1]>shoulder+2:
            msg.data[4] = -50
            print("shoulder__angle:",self.enc_data[1])


        elif self.init == False and self.enc_data[2]<elbow-2:
            msg.data[0] = 50
            print("elbow__angle:",self.enc_data[2])

        elif self.init == False and self.enc_data[2]>elbow+2:
            msg.data[0] = -50
            print("elbow__angle:",self.enc_data[2])

        self.rate.sleep()
        self.motor_data.publish(msg)

    def enc_callback(self,msg):
            self.enc_data = [-msg.data[3],-msg.data[2], -msg.data[5]]
    
    # def spin(self):
    #     while not rospy.is_shutdown():
    #             rate.sleep() 
    def spin(self):
        while not rospy.is_shutdown():
                self.angle_value()
                rate.sleep()       
         
if __name__ == "__main__":
    try:
        rospy.init_node("IK_Test")
        rate=rospy.Rate(10)
        run=IK_IROC()
        run.spin()
    except KeyboardInterrupt:
    # quit
        sys.exit()
