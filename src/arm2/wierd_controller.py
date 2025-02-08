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
        
        outbuff[0] =   (1-msg.buttons[3])*(1-msg.buttons[1])*axes[1] #shoulder
        outbuff[1] = (1-msg.buttons[1])* axes[0] #base rotation
        outbuff[2] =  msg.buttons[3]*axes[1] + msg.buttons[1]*axes[0]  #if outbuff[2] and outbuff[5] are (1,1) or (-1,-1), pitch will happen. If they are (1,-1) or (-1,1), roll will happen
        outbuff[3] =msg.buttons[1]*axes[1] #elbow
        if axes[2]>0.4 or axes[2]<-0.4:     #to not allow command to come till it is moved to a certain extent
            outbuff[4] = axes[2] #gripper
        else:
            outbuff[4] = 0
        outbuff[5] = msg.buttons[3]*axes[1] - msg.buttons[1]*axes[0]

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
        msg.layout.dim[0].label = 'write'
        
        return msg

node = Node()
node.run()
