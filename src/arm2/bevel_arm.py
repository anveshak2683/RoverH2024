#!/usr/bin/env python3

import rospy
import std_msgs.msg as std_msgs
import sensor_msgs.msg as sensor_msgs

class Node:
    def __init__(self):
        
        self.outbuff = [0] *6 
        
        self.pub = rospy.Publisher('stm_write', std_msgs.Int32MultiArray, queue_size=10)
        rospy.init_node('arm_drive')
        rospy.Subscriber('joy_arm', sensor_msgs.Joy, self.joyCallback)

    def joyCallback (self, msg):
        outbuff = [0, 0, 0, 0, 0, 0]
        
        axes = [ int (msg.axes[i] * 0xFF) for i in range(5) ]
        buttons = [ (msg.buttons[1] - msg.buttons[3])*255]
        buttons.append((msg.buttons[0] - msg.buttons[4])*255)
        
        outbuff[0] = -axes[1] #Shoulder
        outbuff[1] = axes[0] #Base
        outbuff[2] = buttons[1]+  buttons[0]   #buttons[0] is 2 buttons, say X and A. If both outbuff[2] and outbuff[5] are positive, roll (say) will happen clockwise; if they are -ve, it will happen anti-clockwise. If [2] is +ve and [5] is -ve, pitch will happen (say) up and vice versa
        outbuff[3] = axes[3] #Elbow 
        outbuff[4] =- axes[2] #Gripper
        outbuff[5] =   buttons[1] -  buttons[0]
        
        self.outbuff = outbuff
        print (self.outbuff)

    def run (self):
        rate = rospy.Rate (50)
        while not rospy.is_shutdown():
            rate.sleep()
            msg = self.createMsg (self.outbuff)
            self.pub.publish (msg)
    
    def createMsg (self, buff):
        # Inititalize the ROS Msg type
        msg = std_msgs.Int32MultiArray()
        
        # Creates a Shallow Copy of outbuff. i.e., changes made to each element of buff will not reflect in msg.data. If we did msg.data=buff, then the elements of msg.data change with the elements of buff.
        msg.data = buff[:] 
        
        msg.layout = std_msgs.MultiArrayLayout()
        msg.layout.data_offset = 0
        
        msg.layout.dim = [ std_msgs.MultiArrayDimension() ]
        msg.layout.dim[0].size = msg.layout.dim[0].stride = len(msg.data)
        msg.layout.dim[-1].label = 'write'
        
        return msg

node = Node()
node.run()
