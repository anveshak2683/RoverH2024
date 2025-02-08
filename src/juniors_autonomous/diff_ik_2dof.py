#!/usr/bin/env python3
import rospy
import numpy as np
from sympy import *
from geometry_msgs.msg import Point
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray,MultiArrayLayout,MultiArrayDimension
import math
import sys
import os

class Differential:


    def __init__(self):

        rospy.init_node('IK_GokulSoham')

        print("INIT RUNNINGGG")
        print("Have patience.........................")
        
        a,b = symbols('a b')

        #link lengths
        l1 = 0.5
        l2 = 0.5

        self.x = l1*cos(a) + l2*cos(a+b)
        self.z = l1*sin(a) + l2*sin(a+b)

        x_prime_a = diff(self.x,a)
        x_prime_b = diff(self.x,b)
        z_prime_a = diff(self.z,a)
        z_prime_b = diff(self.z,b)

        self.J = Matrix(([x_prime_a,x_prime_b],  [z_prime_a,z_prime_b]))
        self.J_inv = self.J**-1

        print("Jacobian ready")
        print("3...")
        print("2...")
        print("1...")

        self.x_vel, self.y_vel, self.z_vel = 0,0,0
        self.vel_matrix = Matrix(([self.x_vel],[self.z_vel]))

        #BUTTONS
        #These are indices for the msg in joy
        self.x_button = 3
        self.y_button = 4
        self.a_button = 1
        self.b_button = 0

        #these are roll, pitch, gripper
        self.motion1, self.motion2, self.gripper = 0,0,0

        #pwm initialisation
        self.pwm = 125

        self.joy_sub=rospy.Subscriber("/joy_arm",Joy,self.joy_callback)
        self.joint_angles_sub=rospy.Subscriber("/enc_arm",Float32MultiArray,self.joint_angles_callback)
        self.joint_angles_vel_pub=rospy.Publisher("/stm_write",Int32MultiArray,queue_size=10)

        


    def joy_callback(self,msg):

        #Check joy values
        self.joy_array=msg.axes
        factor = 0.497
        self.x_vel=self.joy_array[1]*factor
        self.y_vel=-self.joy_array[0]
        self.z_vel=self.joy_array[3]*factor*1.3

        #If both self.motion1 and self.motion2 are positive, roll (say) will happen clockwise; if they are -ve, it will happen anti-clockwise. 
        # If self.motion1 is +ve and self.motion2 is -ve, pitch will happen (say) up and vice versa
        self.motion1 = (-(msg.buttons[self.b_button] - msg.buttons[self.y_button]) - (msg.buttons[self.a_button] - msg.buttons[self.x_button]))*self.pwm
        self.motion2 =  (-(msg.buttons[self.b_button] - msg.buttons[self.y_button]) + (msg.buttons[self.a_button] - msg.buttons[self.x_button]))*self.pwm
        self.gripper = int(self.joy_array[2]*self.pwm)

        #beautiful matrix formed
        self.vel_matrix = Matrix(([self.x_vel],[self.z_vel]))

    def joint_angles_callback(self,msg):

        #Check encoder values for joints from /enc_arm
        self.joint_angles=msg.data
        
        #Factors below are chosen from encoder 
        base_angle=((self.joint_angles[2])) #base is phi
        shoulder_angle=(self.joint_angles[1]/2.055)+90 #shoulder is theta1
        elbow_angle=(self.joint_angles[0]/1)-90 #elbow is theta2

        if (elbow_angle+90) <= -43:
            rospy.logwarn("GURU MEDITATION ERROR: Either elbow is too close to shoulder or rover is about to explode.")

        print()
        print("Angles")
        print("Base: {}, Shoulder: {}, Elbow: {}".format(base_angle, shoulder_angle-90, elbow_angle+90))
        print()
        print()
        self.base=math.radians(base_angle)
        self.shoulder=math.radians(shoulder_angle)
        self.elbow=math.radians(elbow_angle)
        self.output_vel()


    def output_vel(self):
        a,b = symbols('a b')
        vel_output = self.J_inv*self.vel_matrix # Q_vel = J_inv * XYZ_vel
        vel_output = vel_output.subs({a:self.shoulder,b:self.elbow})
        for i in range(2):
            if(vel_output[i]== nan or vel_output[i] == zoo):
                vel_output[i] = 0.0


        theta1_vel = vel_output[0]
        theta2_vel = vel_output[1]


        shoulder_vel = theta1_vel #*60
        elbow_vel = theta2_vel #*30

        speed_factor = 2.5

        base_pwm = int(self.y_vel)*self.pwm
        shoulder_pwm = int(60*shoulder_vel*speed_factor)
        elbow_pwm = int(35*elbow_vel*speed_factor)

        pwm_lst = [base_pwm, shoulder_pwm, elbow_pwm]
        print("Base pwm: {}, shoulder pwm: {}, elbow pwm: {}".format(pwm_lst[0], pwm_lst[1], pwm_lst[2]))

        msg = Int32MultiArray()
        msg.data = [0,0,0,0,0,0]
        #################################
        ### Check output list indices ###
        #################################
        msg.data[3] = elbow_pwm
        msg.data[0] = shoulder_pwm
        msg.data[1] = base_pwm
        msg.data[2] = self.motion1
        msg.data[5] = self.motion2
        msg.data[4] = self.gripper
        msg.layout=MultiArrayLayout()
        msg.layout.data_offset=0
        msg.layout.dim=[MultiArrayDimension()]
        msg.layout.dim[0].size=msg.layout.dim[0].stride=len(msg.data)
        msg.layout.dim[0].label='write'
        self.joint_angles_vel_pub.publish(msg)


def run(args):
    ik_happening = Differential()   
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("When we die, we go bye-bye")

if __name__ == '__main__':
    run(sys.argv)


        


    



