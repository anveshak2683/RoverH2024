#!/usr/bin/env python3

#convention of wheels is [front left, front right, back left, back right]

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
        #Subscribers
        rospy.Subscriber("joy", Joy, self.joyCallback)
        rospy.Subscriber("enc_drive", Float32MultiArray, self.enc_callback)
        rospy.Subscriber("/motion", WheelRpm, self.autonomous_motion_callback, queue_size = 10)

        #Publishers
        self.pwm_pub = rospy.Publisher('motor_pwm', Int32MultiArray, queue_size = 10)
        self.state_pub = rospy.Publisher('state', Bool, queue_size = 10)

        #self.control = ['joystick, autonomous']

        #Standard Code
        self.pwm_msg = Int32MultiArray()
        self.pwm_msg.layout = MultiArrayLayout()
        self.pwm_msg.layout.data_offset = 0
        self.pwm_msg.layout.dim = [ MultiArrayDimension() ]
        self.pwm_msg.layout.dim[0].size = self.pwm_msg.layout.dim[0].stride = len(self.pwm_msg.data)
        self.pwm_msg.layout.dim[0].label = 'write'

        self.forward_btn = 4
        self.parallel_btn = 1
        self.rotinplace_btn = 3
        self.modeupbtn = 7
        self.modednbtn = 6
        #self.steer_unlock_axis = 4
        self.autonomous_btn = 0
        
        self.steer_islocked = True
        self.steering_ctrl_locked = [0,0,0] #gives modes for steering
        self.steering_ctrl_unlocked = [0,0]
        self.steering_ctrl_pwm = [0,0] # axes joy control for steering 
        self.drive_ctrl = [0,0] #drive fb and lr axes
        
        self.mode = 0 # from 0 to 4
        self.state = False
        self.steer_samedir_axis = 2
        self.steer_oppdir_axis = 3
        self.fb_axis = 1 #forward-back
        self.lr_axis = 2 #left-right

        self.prints_per_iter = 3
        self.print_ctrl = self.prints_per_iter

        self.steering_complete = True
        self.s_arr = [25,35,50,75,110] #same as galileo drive multipliers
        self.enc_data = [0,0,0,0]
        self.kp_steer = 1
        self.qsize = 1
        self.vel_prev = queue.Queue(self.qsize) #to implement ledc type control
        self.omega_prev = queue.Queue(self.qsize) #to implement ledc type control
        self.start_time = time.time()
        self.time_thresh = 10
        self.error_thresh = 5
        self.rotinplace = False

        #autonomous params below
        self.crab_rotate = False
        self.autonomous_vel = 0
        self.autonomous_omega = 0

    def autonomous_motion_callback(self,msg):
        if(self.state == True): #process only when in autonomous mode
            self.autonomous_vel = msg.vel #to offset to PWM
            self.autonomous_omega = msg.omega  #to offset to PWM
            self.crab_rotate = msg.hb #hb contains the crab rotation boolean

    def enc_callback(self,msg):
        self.enc_data[0] = (msg.data)[0]
        self.enc_data[1] = (msg.data)[1]
        self.enc_data[2] = (msg.data)[2]
        self.enc_data[3] = (msg.data)[3]

    def joyCallback(self, msg):
        if(self.state == False): #i.e. rover is in manual mode

            if (self.steer_islocked == True):
                # mode up and down
                if(msg.buttons[self.modeupbtn] == 1):
                    if(self.mode < 4):
                        self.mode = self.mode + 1
                if(msg.buttons[self.modednbtn] == 1):
                    if(self.mode >0):
                        self.mode = self.mode - 1
                # differential drive axes
                self.drive_ctrl = [msg.axes[self.fb_axis], msg.axes[self.lr_axis]]
                
             
        if(msg.buttons[self.autonomous_btn] == 1):
            # to switch between manual and autonomous mode
            self.state = not self.state
 
    def spin(self):
        rate = rospy.Rate(13)
        while not rospy.is_shutdown():
            self.main()
            rate.sleep()


    def main(self):
        self.autonomous_control()
        self.drive()
        self.print_ctrl = (self.print_ctrl+1) % self.prints_per_iter
        self.pwm_pub.publish(self.pwm_msg)
        self.state_pub.publish(self.state)
    
    def autonomous_control(self):
        if(self.state == True): #rover is in autonomous mode, listen to commands on relevant topics
            if (self.print_ctrl == 0):    #printing only at certain intervals, to prevent the screen from being filed with data   #print_ctrl is being incremented in main() every time
                print("Rover in Autonomous Mode, Press 'A' to return to manual mode")
                

    def drive(self):

        if(self.steering_complete == True and self.steer_islocked == True):             

            if (self.rotinplace == True):   #if wheels are in rotate in place orientation
                vel = self.s_arr[self.mode] * self.drive_ctrl[1]
                self.pwm_msg.data = [int(vel), -int(vel), int(vel), -int(vel), 0,0,0,0]     #convention at the top

                if (self.print_ctrl == 0):    #printing only at certain intervals, to prevent the screen from being filed with data   #print_ctrl is being incremented in main() every time
                    print("Rotation speed =", int(vel))

            else:

                velocity = 0 #declared earlier so that it doesn't go out of scope
                omega = 0

                if(self.state == False): #without autonomous, where velocity and omega need to be computed 
                    velocity = self.s_arr[self.mode] * self.drive_ctrl[0]
                    omega = self.s_arr[self.mode] * self.drive_ctrl[1]
                else:
                    velocity = self.autonomous_vel
                    omega = self.autonomous_omega

                #for smooth starting and stopping (moving average)
                avg_velocity, avg_omega = 0, 0
                if(self.vel_prev.full() and self.omega_prev.full()):
                    #calculating average of values in queues
                    for i in self.omega_prev.queue:
                        avg_omega = avg_omega + i
                    for j in self.vel_prev.queue:
                        avg_velocity = avg_velocity+j
                    avg_velocity = avg_velocity / self.qsize
                    avg_omega = avg_omega / self.qsize

                    #removing a value from the queues
                    self.vel_prev.get() 
                    self.omega_prev.get()

                #adding a value to each of the queues
                self.vel_prev.put(velocity, True, 2)
                self.omega_prev.put(omega, True, 2)

                print("Velocity:", (avg_velocity//0.001)/1000)   # the math part is to only print upto 2 decimals
                print("Omega: ", (avg_omega//0.001)/1000)
                print("Mode: ", self.mode)
                print()

                self.pwm_msg.data = [int(avg_velocity+avg_omega), -int(avg_velocity-avg_omega), -int(avg_velocity+avg_omega), -int(avg_velocity-avg_omega), 0,0,0,0]      #convention at the top 
            
            #standard code
            self.pwm_msg.layout = MultiArrayLayout()
            self.pwm_msg.layout.data_offset = 0
            self.pwm_msg.layout.dim = [ MultiArrayDimension() ]
            self.pwm_msg.layout.dim[0].size = self.pwm_msg.layout.dim[0].stride = len(self.pwm_msg.data)
            self.pwm_msg.layout.dim[0].label = 'write'

        else:
            pass


    
if __name__ == '__main__':
    run = Drive()
    run.spin()



