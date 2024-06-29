import rospy
import std_msgs.msg as std_msgs
import sensor_msgs.msg as sensor_msgs
import math

class Node:
    def __init__(self):
        
        self.outbuff = [0] * 6
        rospy.init_node('arm_drive')

        self.pub = rospy.Publisher('stm_write', std_msgs.Int32MultiArray, queue_size=10)

        rospy.Subscriber('joy_arm', sensor_msgs.Joy, self.joyCallback)
        rospy.Subscriber('auto_arm_signals', std_msgs.Int32MultiArray, self.auto_callback)
        rospy.Subscriber("/enc_drive", std_msgs.Float32MultiArray, self.joint_angles_callback)

        self.autonomous_mode = False
        self.diff_mode = False
        self.auto_outbuff = [0,0,0,0,0,0]
        self.x_vel, self.y_vel, self.z_vel = 0, 0, 0
        self.joy_array = [0,0,0,0,0,0]

    def auto_callback(self, msg):
        self.auto_outbuff = msg.data

    def joint_angles_callback(self, msg):
        self.joint_angles = msg.data

    def joyCallback (self, msg):
        
        if (self.autonomous_mode==False and self.diff_mode==False):
            outbuff = [0, 0, 0, 0, 0, 0]       
            #HASA
            outbuff[0] = - int(msg.axes[1] * 0xFF)
            outbuff[1] = - int(msg.axes[0] * 0xFF)
            outbuff[2] = int(msg.axes[3] * 0xFF)
            outbuff[4] = - int(msg.axes[2] * 0xFF)
            outbuff[3] = int(msg.axes[7] * 255)
            outbuff[5] = - int(msg.axes[6] * 255)

            self.outbuff = outbuff
        
        elif (self.autonomous_mode==False and self.diff_mode==True):
            self.joy_array = msg.axes

        if msg.buttons[0] == 1:
            self.autonomous_mode = not self.autonomous_mode
            self.diff_mode = False
        if msg.buttons[1] == 1:
            self.diff_mode = not self.diff_mode
            self.autonomous_mode = False
    
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
    
    def spin(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            rate.sleep()
            if self.autonomous_mode == True and self.diff_mode == False:
                print("In Autonomous Mode")
                msg = self.createMsg(self.auto_outbuff)
                self.pub.publish(msg)
            elif self.autonomous_mode == False and self.diff_mode == False:
                msg = self.createMsg(self.outbuff)
                self.pub.publish (msg)
            elif self.autonomous_mode == False and self.diff_mode == True:
                factor = 1
                self.x_vel = self.joy_array[0]*factor
                self.y_vel = self.joy_array[1]*factor*2
                self.z_vel = self.joy_array[2]*factor
                self.calc()     #publishes in a function called by calc()

    def joint_angle_vel(self, phi_vel, theta1_vel, theta2_vel):
        max_rpm = 30
        max_angular_vel = math.pi
        base_vel = 15*phi_vel  # gear ratio 15:1
        shoulder_vel = 15*theta1_vel
        elbow_vel = -20*theta2_vel
        factor = 255/600*2  # 30*20

        base_pwm = int(base_vel*factor)
        shoulder_pwm = int(shoulder_vel*factor)
        elbow_pwm = int(elbow_vel*factor)

        msg = std_msgs.Int32MultiArray()
        msg.data = [0, 0, 0, 0, 0, 0]
        msg.data[0] = elbow_pwm
        msg.data[4] = shoulder_pwm
        msg.data[1] = base_pwm
        msg.layout =std_msgs.MultiArrayLayout()
        msg.layout.data_offset = 0
        msg.layout.dim = [std_msgs.MultiArrayDimension()]
        msg.layout.dim[0].size = msg.layout.dim[0].stride = len(msg.data)
        msg.layout.dim[0].label = 'write'
        self.pub.publish(msg)
        print("Published Velocities", msg.data)

    def position(self, r, theta, phi):
        x = r*math.cos(theta)*math.cos(phi)
        y = r*math.cos(theta)*math.sin(phi)
        z = r*math.sin(theta)
        return (x, y, z)

    def calc(self):
        base_angle = -self.joint_angles[3]  # base is phi
        shoulder_angle = self.joint_angles[2]+90  # shoulder is theta1
        elbow_angle = -self.joint_angles[5]+90  # elbow is theta2
        phi = math.radians(base_angle)
        theta1 = math.radians(shoulder_angle)
        theta2 = math.radians(elbow_angle)
        l1 = 0.4
        l2 = 0.35
        r = (l1**2+l2**2-2*l1*l2*math.cos(theta2))**0.5
        if (l2-l1*math.cos(theta2)) == 0:
            theta = math.pi/2+theta1+theta2-math.pi
        else:
            theta = math.atan(l1*math.sin(theta2) /
                              (l2-l1*math.cos(theta2)))+theta1+theta2-math.pi
        # now we will compute the x,y,z positions
        # x is +ve along direction rover is facing
        # y is in the same plane as x and in 90 degrees left to the direction of x
        # z is in perpendicular to both x and y and is facing upwards
        # basically the general convention of r,theta, phi is used
        (x, y, z) = self.position(r, theta, phi)
        ####################
        # now compute r_vel and theta_vel
        x_vel, y_vel, z_vel = self.x_vel, self.y_vel, self.z_vel
        print("Hello Tanish", x_vel, y_vel, z_vel)
        r_vel = (x_vel+y_vel+z_vel)/r
        theta_vel = (z_vel*(x**2+y**2)-x*z*x_vel-y*z*y_vel) / \
            (r**3*math.cos(theta))

        # now compute theta1_vel,theta2_vel,phi_vel
        phi_vel = (x*y_vel-y*x_vel)/(r*math.cos(theta))**2
        theta2_vel = r*r_vel/(l1*l2*math.sin(theta2))
        theta1_vel = theta_vel-(l2**2-l1*l2*math.cos(theta2))*theta2_vel/r**2
        # in radians per second
        self.joint_angle_vel(phi_vel, theta1_vel, theta2_vel)

if __name__ == '__main__':
    node = Node()
    node.spin()
