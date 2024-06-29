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
        rospy.Subscriber("motion", WheelRpm, self.autonomous_motion_callback)
        rospy.Subscriber("joy", Joy, self.joyCallback)
        rospy.Subscriber("enc_auto", Float32MultiArray, self.enc_callback)
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
        self.steer_unlock_axis = 4
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
        self.lr_axis = 0 #left-right

        self.prints_per_iter = 3
        self.print_ctrl = self.prints_per_iter

        self.steering_complete = True
        self.s_arr = [25,35,50,75,110] #same as galileo drive multipliers
        self.enc_data = [0,0,0,0]
        self.kp_steer = 1
        self.qsize = 5
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
            self.autonomous_vel = 127- msg.vel #to offset to PWM
            self.autonomous_omega = 127 -msg.omega  #to offset to PWM
            self.crab_rotate = msg.hb #hb contains the crab rotation boolean

    def enc_callback(self,msg):
        self.enc_data[0] = (msg.data)[0]
        self.enc_data[1] = (msg.data)[1]
        self.enc_data[2] = (msg.data)[2]
        self.enc_data[3] = (msg.data)[3]

    def joyCallback(self, msg):
        
        if(self.state == False): #i.e. rover is in manual mode

            if (self.steer_islocked == True):
                # steering buttons for locked mode (functionality is in the name)
                self.steering_ctrl_locked = [msg.buttons[self.forward_btn], msg.buttons[self.parallel_btn], msg.buttons[self.rotinplace_btn]]
                # mode up and down
                if(msg.buttons[self.modeupbtn] == 1):
                    if(self.mode < 4):
                        self.mode = self.mode + 1
                if(msg.buttons[self.modednbtn] == 1):
                    if(self.mode >0):
                        self.mode = self.mode - 1
                # differential drive axes
                self.drive_ctrl = [msg.axes[self.fb_axis], msg.axes[self.lr_axis]]
                
            elif (self.steer_islocked == False):
                # buttons for steering unlocked (relative +45 and -45)
                self.steering_ctrl_unlocked = [msg.buttons[self.forward_btn],msg.buttons[self.parallel_btn]] 
                # steering axes for all 4 wheels at once
                self.steering_ctrl_pwm = [msg.axes[self.steer_samedir_axis], msg.axes[self.steer_oppdir_axis]]

            if (msg.axes[self.steer_unlock_axis] == -1.0):
                # to lock/unlock steering
                self.steer_islocked = not self.steer_islocked
             
        if(msg.buttons[self.autonomous_btn] == 1):
            # to switch between manual and autonomous mode
            self.state = not self.state
 
    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.main()
            rate.sleep()
            self.pwm_pub.publish(self.pwm_msg)
            self.state_pub.publish(self.state)


    def main(self):
        self.autonomous_control()
        self.steering()
        self.drive()
        self.print_ctrl = (self.print_ctrl+1) % self.prints_per_iter
    
    def autonomous_control(self):
        if(self.state == True): #rover is in autonomous mode, listen to commands on relevant topics
            if (self.print_ctrl == 0):    #printing only at certain intervals, to prevent the screen from being filed with data   #print_ctrl is being incremented in main() every time
                print("Rover in Autonomous Mode, Press 'A' to return to manual mode")
                

    def steering(self):

        if(self.state == False): #default, when not autonomous

            if (self.steer_islocked == True):

                if(self.steering_ctrl_locked[0] == 1):
                    print() #readability (basically line spaces)
                    print("Rotating steering forward")
                    print()
                    self.steering_complete = False
                    self.rotinplace = False
                    self.start_time = time.time()
                    self.steer([0,0,0,0],[0,0,0,0],1)  #initial_angle = 0, final_angle, mode = 1 for absolute

                elif(self.steering_ctrl_locked[1] == 1):
                    print()
                    print("Rotating steering perpendicular to rover")
                    print()
                    self.steering_complete = False
                    self.rotinplace = False
                    self.start_time = time.time()
                    self.steer([0,0,0,0],[90,90,90,90],1) #initial angle, final angle, mode=1 for absolute 
                
                elif(self.steering_ctrl_locked[2] == 1):
                    print()
                    print("Rotating steering for in place rotation")
                    print()
                    self.rotinplace = True
                    self.steering_complete = False
                    self.start_time = time.time()
                    self.steer([0,0,0,0],[45,-45,-45,45],1) #rotating in place, mode=1 for absolute     #convention at the top
            
            elif (self.steer_islocked == False):

                enc_data_new = copy.deepcopy(self.enc_data) # to create a deep copy of enc_data array, not a pointer equivalence

                if (self.steering_ctrl_unlocked[0] == 1):
                    print()
                    print("Turning steering clockwise by 45 deg")
                    print()
                    self.steering_complete = False
                    self.rotinplace = False
                    self.start_time = time.time()
                    self.steer(enc_data_new,[45,45,45,45],0) #initial angle, final angle, mode=0 for relative

                elif (self.steering_ctrl_unlocked[1] == 1):
                    print()
                    print("Turning steering anti-clockwise by 45 deg")
                    print()
                    self.steering_complete = False
                    self.rotinplace = False
                    self.start_time = time.time()
                    self.steer(enc_data_new,[-45,-45,-45,-45],0) #initial angle, final angle, mode=0 for relative

                elif (self.steering_ctrl_pwm[0]!=0 and self.steering_ctrl_pwm[1] == 0):
                    self.steering_complete = False
                    self.rotinplace = False
                    self.start_time = time.time()
                    temp = int(self.s_arr[self.mode] * self.steering_ctrl_pwm[0])
                    self.pwm_msg.data = [0,0,0,0,temp,temp,temp,temp]
                    print("Encoder angles:-", self.enc_data, end = "       ") 
                    print("All wheels -> same direction.")

                elif (self.steering_ctrl_pwm[1] != 0 and self.steering_ctrl_pwm[0] == 0):
                    self.steering_complete = False
                    self.rotinplace = False
                    self.start_time = time.time()
                    temp = int(self.s_arr[self.mode] * self.steering_ctrl_pwm[1])
                    self.pwm_msg.data = [0,0,0,0,temp,temp,-temp,-temp]     #convention at the top
                    print("Encoder angles:-", self.enc_data, end = "       ") 
                    print("Front and back wheels -> opposite direction.")

                else:
                    self.pwm_msg.data = [0,0,0,0,0,0,0,0]
                    if (self.print_ctrl == 0):    #printing only at certain intervals, to prevent the screen from being filed with data
                        print("Steering is unlocked, lock it to perform drive.")

        else: #case when crab_bool is set to True or False

            if(self.crab_rotate == False): #all wheels forward 
                print() #readability (basically line spaces)
                print("Rotating steering forward")
                print()
                self.steering_complete = False
                self.rotinplace = False
                self.start_time = time.time()
                self.steer([0,0,0,0],[0,0,0,0],1)  #initial_angle = 0, final_angle, mode = 1 for absolute

            else:
                print()
                print("Rotating steering perpendicular to rover")
                print()
                self.steering_complete = False
                self.rotinplace = False
                self.start_time = time.time()
                self.steer([0,0,0,0],[90,90,90,90],1) #initial angle, final angle, mode=1 for absolute 



        self.steering_complete = True
        self.start_time = time.time()

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

                self.pwm_msg.data = [int(avg_velocity+avg_omega), int(avg_velocity-avg_omega), int(avg_velocity+avg_omega), int(avg_velocity-avg_omega), 0,0,0,0]      #convention at the top 
            
            #standard code
            self.pwm_msg.layout = MultiArrayLayout()
            self.pwm_msg.layout.data_offset = 0
            self.pwm_msg.layout.dim = [ MultiArrayDimension() ]
            self.pwm_msg.layout.dim[0].size = self.pwm_msg.layout.dim[0].stride = len(self.pwm_msg.data)
            self.pwm_msg.layout.dim[0].label = 'write'

        else:
            pass


    def steer(self, initial_angles, final_angles, mode):    #mode is not to be confused with the mode variable for speed control

        if(mode == 0):      # relative (encoder will go <final_angle> away from its initial position)
            pwm = [0,0,0,0]
            while( (abs(self.enc_data[0] - initial_angles[0]) < abs(final_angles[0])-self.error_thresh or abs(self.enc_data[1] - initial_angles[1]) < abs(final_angles[1])-self.error_thresh or abs(self.enc_data[2] - initial_angles[2]) < abs(final_angles[2])-self.error_thresh or abs(self.enc_data[3] - initial_angles[3]) < abs(final_angles[3])-self.error_thresh)  and time.time() - self.start_time <= self.time_thresh): #time_thresh = 10s   #error_thresh = 5 degrees

                if(int(time.time() - self.start_time) * 10 % 2 == 0):  #printing only at certain intervals, to prevent the screen from being filed with data
                    print("Enc_data:- ", self.enc_data, end = "     ")
                    print("Final angles:- ", list(map(add, initial_angles, final_angles)))
                
                for i in range(4):
                    if (abs(self.enc_data[i] - initial_angles[i]) < abs(final_angles[i])-self.error_thresh):
                        pwm[i] = int(self.kp_steer*(final_angles[i]-(self.enc_data[i]-initial_angles[i])))
                    else:
                        pwm[i] = 0

                self.pwm_msg.data = [0,0,0,0,pwm[0],pwm[1],pwm[2],pwm[3]]

                #standard code
                rate = rospy.Rate(10)
                rate.sleep()
                self.pwm_pub.publish(self.pwm_msg)
                                

        elif(mode == 1):        # absolute (encoder will become equal to the final angle specified)
            pwm = [0,0,0,0]
            while( (abs(self.enc_data[0] - final_angles[0]) > self.error_thresh or abs(self.enc_data[1] - final_angles[1]) > self.error_thresh or abs(self.enc_data[2] - final_angles[2]) > self.error_thresh or abs(self.enc_data[3] - final_angles[3]) > self.error_thresh) and time.time() - self.start_time <= self.time_thresh):   #error_thresh = 5 degrees

                if(int(time.time() - self.start_time) * 10 % 2 == 0):   #printing only at certain intervals, to prevent the screen from being filed with data
                    print("Enc_data:- ", self.enc_data, end = "     ")
                    print("Final angles:- ", final_angles)
                
                for i in range(4):
                    if (abs(self.enc_data[i] - final_angles[i]) > self.error_thresh):
                        pwm[i] = -int(self.kp_steer*(final_angles[i]-self.enc_data[i]))
                    else:
                        pwm[i] = 0
                
                self.pwm_msg.data = [0,0,0,0,pwm[0],pwm[1],pwm[2],pwm[3]]
                
                '''
                self.pwm_msg.data = [0,0,0,0,int(self.kp_steer*(final_angles[0]-self.enc_data[0])),int(self.kp_steer*(final_angles[1]-self.enc_data[1])),int(self.kp_steer*(final_angles[2] - self.enc_data[2])),int(self.kp_steer*(final_angles[3] - self.enc_data[3]))]
                '''
                rate = rospy.Rate(10)
                rate.sleep()
                self.pwm_pub.publish(self.pwm_msg)
        
        print()
        print("***Steering Complete***")
        print()
    
if __name__ == '__main__':
    run = Drive()
    run.spin()



