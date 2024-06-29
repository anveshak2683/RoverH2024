import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray,MultiArrayLayout,MultiArrayDimension
from std_msgs.msg import Int32MultiArray, Bool
import sys
import math
from sensor_msgs.msg import Joy

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
        rospy.Subscriber('joy_arm', Joy, self.joyCallback)
        self.kpwm = [0.01,0.01,0.01]
        self.enc_data = [0,0,0]
        self.arm_goal_coming_from_tanish = False
        self.goal = [0,0.35,0.4]
        self.q1, self.q2, self.q3 = 0,0,0
        self.pwm_limit = 50

    def joyCallback(self,msg): 
        self.goal[0] = self.goal[0]-self.kpwm[0]*msg.axes[2]
        self.goal[1] = self.goal[1]+self.kpwm[1]*msg.axes[3]
        self.goal[2] = self.goal[2]+self.kpwm[2]*msg.axes[1]

    def angle_value(self):
        a1 = 0.4
        a2 = 0.35
        x = self.goal[0]
        y = self.goal[1]
        z = self.goal[2]
        print(f"Coordinates: {x},{y},{z}")
        if(abs((x**2+y**2+z**2-a1**2-a2**2)/(2*a1*a2)) <1):
            self.q3 = math.pi/2
            self.q2 = -math.acos((x**2+y**2+z**2-a1**2-a2**2)/(2*a1*a2))
            self.q1 = math.atan(z/math.sqrt(x**2+y**2)) + math.atan((-a2*math.sin(self.q2))/(a1+a2*math.cos(self.q2)))
        if(y != 0):
            self.q3 = math.atan(x/y)

        
        theta1 = self.q3
        theta2 = math.pi/2 - self.q1
        theta3 = math.pi/2 + self.q2

       # print(np.arccos(self.pz/(2*k)))
        base = (180*theta1)/math.pi
        shoulder = (180*theta2)/math.pi
        elbow = (180*theta3)/math.pi
        print(f"Goal: Base = {base}, Shoulder = {shoulder}, Elbow = {elbow}")  

        print(f"Current: Base = {self.enc_data[0]}, Shoulder = {self.enc_data[1]}, Elbow = {self.enc_data[2]}")
        msg=Int32MultiArray()
        msg.data=[0,0,0,0,0,0]

        msg.layout = MultiArrayLayout()
        msg.layout.data_offset = 0
        msg.layout.dim = [MultiArrayDimension()]
        msg.layout.dim[0].size = msg.layout.dim[0].stride = len(msg.data)
        msg.layout.dim[0].label = 'write'
  	
        msg.data = [0,0,0,0,0,0]

        if self.enc_data[0]<base-2:
            msg.data[2] = 50

        elif self.enc_data[0]>base+2:
            msg.data[2] = -50


        if self.enc_data[1]<shoulder-2:
            msg.data[4] = -50

        elif self.enc_data[1]>shoulder+2:
            msg.data[4] = 50


        if self.enc_data[2]<elbow-2:
            msg.data[0] = -50

        elif self.enc_data[2]>elbow+2:
            msg.data[0] = 50
            
        if abs(self.enc_data[0] - base) <=3 and abs(self.enc_data[1] - shoulder) <=3 and abs(self.enc_data[2] - elbow) <=3:
           print("Goal Reached")
        else:
           print("Goal Not Reached")

        self.rate.sleep()
        self.motor_data.publish(msg)

    def enc_callback(self,msg):
            self.enc_data = [-msg.data[3],msg.data[2], msg.data[5]]
    
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
