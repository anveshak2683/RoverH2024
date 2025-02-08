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




class Differential:


    def __init__(self):

        rospy.init_node('IK_GokulSoham')

        print("INIT RUNNINGGG")
        print("This is a COPY COPY COPY")
        print("Have patience.........................")
        
        a,b,c = symbols('a b c')

        #link lengths
        l1 = 0
        l2 = 0.597 #shoulder motor to elbow motor
        l3 = 0.546 #elbow motor to gripper

        #Forward kinematics equations and formulation of jacobian
        self.x = (cos(a)*cos(b+c)*l3 + cos(a)*cos(b)*l2)
        self.y = sin(a)*cos(b+c)*l3 + sin(a)*cos(b)*l2
        self.z = (sin(b+c)*l3 + sin(b)*l2 + l1)

        self.start_angles =np.array([0, math.pi/2, 3*math.pi/2])

        #Jacobian
        
        x_prime_a = diff(self.x,a)
        x_prime_b = diff(self.x,b)
        x_prime_c = diff(self.x,c)
        y_prime_a = diff(self.y,a)
        y_prime_b = diff(self.y,b)
        y_prime_c = diff(self.y,c)
        z_prime_a = diff(self.z,a)
        z_prime_b = diff(self.z,b)
        z_prime_c = diff(self.z,c)

        self.J = Matrix(([x_prime_a,x_prime_b,x_prime_c], [y_prime_a,y_prime_b,y_prime_c], [z_prime_a,z_prime_b,z_prime_c]))

        self.J_inv = self.J**-1
        print("Jacobian ready")
        print("3...")
        print("2...")
        print("1...")

        self.x_vel, self.y_vel, self.z_vel = 0,0,0
        self.vel_matrix = Matrix(([self.x_vel],[self.y_vel],[self.z_vel]))


        self.joy_sub=rospy.Subscriber("/joy_arm",Joy,self.joy_callback)
        self.joint_angles_sub=rospy.Subscriber("/enc_auto",Float32MultiArray,self.joint_angles_callback)
        self.joint_angles_vel_pub=rospy.Publisher("/stm_write",Int32MultiArray,queue_size=10)

   
    def joy_callback(self,msg):

        #Check joy values
        self.joy_array=msg.axes
        factor = 0.5
        self.x_vel=-self.joy_array[1]*factor*1.5
        self.y_vel=self.joy_array[0]*factor*1.5
        self.z_vel=-self.joy_array[3]*factor*1.5
        self.vel_matrix = Matrix(([self.x_vel],[self.y_vel],[self.z_vel]))

    #Determining angular velocities for differential kinematics to occur
    def joint_angles_callback(self,msg):

        #Check encoder values for joints from /enc_auto 
        self.joint_angles=msg.data
        
        #Factors below are chosen from encoder 
        base_angle=360-((self.joint_angles[0]/1.4778)) #base is phi
        shoulder_angle=(self.joint_angles[4]/2.93)+90 #shoulder is theta1
        elbow_angle=(self.joint_angles[1]/1.33)+270 #elbow is theta2
        print()
        print("Angles")
        print("Base: {}, Shoulder: {}, Elbow: {}".format(-base_angle+2*math.pi, shoulder_angle-90, elbow_angle-270))
        print()
        print()
        self.base=math.radians(base_angle)
        self.shoulder=math.radians(shoulder_angle)
        self.elbow=math.radians(elbow_angle)
        self.output_vel()

    #Determining output velocities to be published
    def output_vel(self):
        a,b,c = symbols('a b c')
        vel_output = self.J_inv*self.vel_matrix # Q_vel = J_inv * XYZ_vel
        vel_output = vel_output.subs({a:self.base,b:self.shoulder,c:self.elbow})
        for i in range(3):
            if(vel_output[i]== nan or vel_output[i] == zoo):
                vel_output[i] = 0.0

        phi_vel = vel_output[0]
        theta1_vel = vel_output[1]
        print("theta 1: ", theta1_vel)
        theta2_vel = vel_output[2]
        

        #Check gear ratios 
        base_vel = phi_vel #*10
        shoulder_vel = theta1_vel #*60
        elbow_vel = theta2_vel #*30

        #Maintain gear ratios(arbitrary) for pwm values and appropriate speed factor multiplied for best motion

        speed_factor = 1.5

        base_pwm = int(60*base_vel*speed_factor)
        shoulder_pwm = int(100*shoulder_vel*speed_factor)
        elbow_pwm = int(50*elbow_vel*speed_factor)

        

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

        msg.layout=MultiArrayLayout()
        msg.layout.data_offset=0
        msg.layout.dim=[MultiArrayDimension()]
        msg.layout.dim[0].size=msg.layout.dim[0].stride=len(msg.data)
        msg.layout.dim[0].label='write'
        self.joint_angles_vel_pub.publish(msg)
        print("vel published, be happy")


def runner(args):
    ik_happening = Differential()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")

if __name__ == '__main__':
    runner(sys.argv)









