import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray,MultiArrayLayout,MultiArrayDimension
import math
import sys




class differential_ik:
    def __init__(self):
        rospy.init_node('Tanish_IK_Node')
        self.joy_sub=rospy.Subscriber("/joy_arm",Joy,self.joy_callback)
        self.joint_angles_sub=rospy.Subscriber("/enc_drive",Float32MultiArray,self.joint_angles_callback)
        self.joint_angles_vel_pub=rospy.Publisher("/stm_write",Int32MultiArray,queue_size=10)
        self.x_vel, self.y_vel, self.z_vel = 0,0,0

    def joy_callback(self,msg):
        self.joy_array=msg.axes
        factor=1
        self.x_vel=self.joy_array[0]*factor
        self.y_vel=self.joy_array[1]*factor*2
        self.z_vel=self.joy_array[2]*factor
        print("Joy Callback")

    def joint_angles_callback(self,msg):
        self.joint_angles=msg.data
        base_angle=-self.joint_angles[3] #base is phi
        shoulder_angle=self.joint_angles[2]+90 #shoulder is theta1
        elbow_angle=-self.joint_angles[5]+90 #elbow is theta2
        phi=math.radians(base_angle)
        theta1=math.radians(shoulder_angle)
        theta2=math.radians(elbow_angle)
        l1=0.4
        l2=0.35
        r=(l1**2+l2**2-2*l1*l2*math.cos(theta2))**0.5
        if (l2-l1*math.cos(theta2))==0:
            theta=math.pi/2+theta1+theta2-math.pi
        else:
            theta=math.atan(l1*math.sin(theta2)/(l2-l1*math.cos(theta2)))+theta1+theta2-math.pi
        # now we will compute the x,y,z positions
        # x is +ve along direction rover is facing
        # y is in the same plane as x and in 90 degrees left to the direction of x
        # z is in perpendicular to both x and y and is facing upwards
        # basically the general convention of r,theta, phi is used
        (x,y,z)=self.position(r,theta,phi)
        ####################
        #now compute r_vel and theta_vel
        x_vel,y_vel,z_vel=self.x_vel,self.y_vel,self.z_vel
        print("Hello Tanish", x_vel,y_vel,z_vel) 
        r_vel=(x_vel+y_vel+z_vel)/r
        theta_vel=(z_vel*(x**2+y**2)-x*z*x_vel-y*z*y_vel)/(r**3*math.cos(theta))

        # now compute theta1_vel,theta2_vel,phi_vel
        phi_vel=(x*y_vel-y*x_vel)/(r*math.cos(theta))**2
        theta2_vel=r*r_vel/(l1*l2*math.sin(theta2))
        theta1_vel=theta_vel-(l2**2-l1*l2*math.cos(theta2))*theta2_vel/r**2
        self.joint_angle_vel(phi_vel,theta1_vel,theta2_vel) # in radians per second


        # publishing joint angle velocities 
    def joint_angle_vel(self,phi_vel,theta1_vel,theta2_vel):
        max_rpm=30
        max_angular_vel=math.pi
        base_vel=15*phi_vel # gear ratio 15:1
        shoulder_vel=15*theta1_vel
        elbow_vel=-20*theta2_vel
        factor=255/600*2 #30*20

        base_pwm=int(base_vel*factor)
        shoulder_pwm=int(shoulder_vel*factor)
        elbow_pwm=int(elbow_vel*factor)
        

        msg = Int32MultiArray()
        msg.data = [0,0,0,0,0,0]
        msg.data[0] = elbow_pwm
        msg.data[4] = shoulder_pwm
        msg.data[1] = base_pwm
        msg.layout=MultiArrayLayout()
        msg.layout.data_offset=0
        msg.layout.dim=[MultiArrayDimension()]
        msg.layout.dim[0].size=msg.layout.dim[0].stride=len(msg.data)
        msg.layout.dim[0].label='write'
        self.joint_angles_vel_pub.publish(msg)
        print("Published Velocities", msg.data)

    def position(self,r,theta,phi):
        x=r*math.cos(theta)*math.cos(phi)
        y=r*math.cos(theta)*math.sin(phi)
        z=r*math.sin(theta)
        return (x,y,z)
        
def main(args):
    td=differential_ik()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")

if __name__ == '__main__':
    main(sys.argv)

    

    





        
