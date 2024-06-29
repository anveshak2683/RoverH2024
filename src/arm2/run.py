#!/usr/bin/env python

import rospy
import std_msgs.msg as std_msgs
import sensor_msgs.msg as sensor_msgs


class Node:
    def __init__(self):
        
        self.outbuff = [0] * 6
        
        self.pub = rospy.Publisher('stm_write', std_msgs.Int32MultiArray, queue_size=10)
        rospy.init_node('arm_drive')
        rospy.Subscriber('joy_arm', sensor_msgs.Joy, self.joyCallback)
        self.autonomous_mode = False
        self.auto_outbuff = [0,0,0,0,0,0]
        rospy.Subscriber('auto_arm_signals', std_msgs.Int32MultiArray, self.auto_callback)

    def auto_callback(self, msg):
        self.auto_outbuff = msg.data
    
    def joyCallback_0 (self, msg):
        self.outbuff = [ int (msg.axes[i] * 0xCF) for i in range(4) ]
        self.outbuff += [ (msg.buttons[i] - msg.buttons[i+2]) * 0xFF for i in range(4, 6) ]

    def joyCallback (self, msg):
        outbuff = [0, 0, 0, 0, 0, 0]
        
       # outbuff = [ int (msg.axes[i] * 0xFF) for i in range(4) ]
       # outbuff += [ (msg.buttons[i] - msg.buttons[i+2]) * 0xFF for i in range(4, 6) ]
        
        axes = [ int (msg.axes[i] * 0x32) for i in range(5) ]
        buttons = [ int((msg.axes[7])*255)]
        buttons.append(int((msg.axes[6])*255))
       
       #HASA
        outbuff[0] = - axes[1]
        outbuff[1] = - axes[0]
        outbuff[2] = axes[3]
        outbuff[4] = axes[2]
        outbuff[3] = buttons[0]
        outbuff[5] = - buttons[1]

        if msg.buttons[0] == 1:
            self.autonomous_mode = not self.autonomous_mode

        
        self.outbuff = outbuff

    def run (self):
        rate = rospy.Rate (50)
        while not rospy.is_shutdown():
            rate.sleep()
            if self.autonomous_mode == True:
                print("In Autonomous Mode")
                msg = self.createMsg (self.auto_outbuff)
                self.pub.publish(msg)
            else:
                msg = self.createMsg (self.outbuff)
                self.pub.publish (msg)
    
    def createMsg (self, buff):
        msg = std_msgs.Int32MultiArray()
        msg.data = buff[:]
        
        msg.layout = std_msgs.MultiArrayLayout()
        msg.layout.data_offset = 0
        
        msg.layout.dim = [ std_msgs.MultiArrayDimension() ]
        msg.layout.dim[0].size = msg.layout.dim[0].stride = len(msg.data)
        msg.layout.dim[0].label = 'write'
        if self.autonomous_mode == False:
            print (self.outbuff)
        else:
            print (self.auto_outbuff)
         
        return msg

node = Node()
node.run()

