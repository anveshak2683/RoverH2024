'''
This code was made to check which index of motor_pwm corresponds to which drive/steering motor, mostly to play around with the speed of any individual motor.
Do not pay any attention to the joystick axes or to the sign of the published integers, just check which axis of motor_pwm has a number when a certain wheel moves.

- written by Amogh, used code written by Pranav
'''

#!/usr/bin/env python3
import rospy
import copy
from sensor_msgs.msg import Joy
import time
from std_msgs.msg import Int8
from std_msgs.msg import Float32
from std_msgs.msg import Int32MultiArray, MultiArrayLayout, MultiArrayDimension, Float32MultiArray, Bool
import queue
from traversal.msg import WheelRpm
from operator import add

class Drive:
    def __init__(self):
        rospy.init_node("drive_arc")
        self.pwm_pub = rospy.Publisher('motor_pwm', Int32MultiArray, queue_size = 10)
        rospy.Subscriber("joy", Joy, self.joyCallback)

        self.drive_mode = True
        self.dr_pwm = [0,0,0,0]
        self.str_pwm = [0,0,0,0]

        self.prints_per_iter = 1
        self.print_ctrl = self.prints_per_iter
        
        self.pwm_msg = Int32MultiArray()
        self.pwm_msg.layout = MultiArrayLayout()
        self.pwm_msg.layout.data_offset = 0
        self.pwm_msg.layout.dim = [ MultiArrayDimension() ]
        self.pwm_msg.layout.dim[0].size = self.pwm_msg.layout.dim[0].stride = len(self.pwm_msg.data)
        self.pwm_msg.layout.dim[0].label = 'write'

    def joyCallback(self, msg):
        if (self.drive_mode == True):
            self.dr_pwm = [msg.axes[1], msg.axes[0], msg.axes[2], msg.axes[3]]
        else:
            self.str_pwm = [msg.axes[1], msg.axes[0], msg.axes[2], msg.axes[3]]

        if (msg.buttons[4]==1):
            self.drive_mode = not self.drive_mode

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.main()
            rate.sleep()
            if (self.print_ctrl == 0):
                if (self.drive_mode == True):
                    print("Drive motors:-")
                else:
                    print("Steering motors:-")
                print("Sending",self.pwm_msg.data,"to motor_pwm.")
                print()
            self.pwm_pub.publish(self.pwm_msg)

    def main(self):
        self.drive()
        self.print_ctrl = (self.print_ctrl+1) % self.prints_per_iter
    
    def drive(self):
        if (self.drive_mode == True):
            self.pwm_msg.data = [int(30*self.dr_pwm[0]), int(30*self.dr_pwm[1]), int(30*self.dr_pwm[2]), int(30*self.dr_pwm[3]), 0, 0, 0, 0]
        else:
            self.pwm_msg.data = [0, 0, 0, 0, int(255*self.str_pwm[0]), int(255*self.str_pwm[1]), int(255*self.str_pwm[2]), int(255*self.str_pwm[3])]

        self.pwm_msg.layout = MultiArrayLayout()
        self.pwm_msg.layout.data_offset = 0
        self.pwm_msg.layout.dim = [ MultiArrayDimension() ]
        self.pwm_msg.layout.dim[0].size = self.pwm_msg.layout.dim[0].stride = len(self.pwm_msg.data)
        self.pwm_msg.layout.dim[0].label = 'write'

if __name__ == '__main__':
    run = Drive()
    run.spin()
