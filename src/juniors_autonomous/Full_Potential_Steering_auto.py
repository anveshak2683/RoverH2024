#!/usr/bin/env python3

#no change in sign is to be made to adjust for change in hardware, make sure that the array corresponds to [fl_dr,fr_dr,bl_dr,br_dr,fl_str,fr_str,bl_str,br_str] and + corresponds to front (for drive) or right (for steering) for each and every wheel in the rostopic motor_pwm. Any inconsistency should be corrected in the .ino file only!
#encoder sign cannot be adjusted for in the .ino file or the hardware, due to hardware inconsistencies in the 6 channels, so make sure to keep right as +ve for that respective encoder in the array in this code (by changing the signs in the encoder callback). Also make sure that the array corresponds to [fl,fr,bl,br]

import rospy
import copy
from sensor_msgs.msg import Joy
import time
from std_msgs.msg import Int8
from std_msgs.msg import Float32
from std_msgs.msg import Int32MultiArray, MultiArrayLayout, MultiArrayDimension, Float32MultiArray, Bool
import queue
from operator import add
#from navigation.msg import WheelRpm
from traversal.msg import WheelRpm

class Drive:
    def __init__(self):
        self.init_dir =[1,1,1,1,1,1,1,1]       #to be changed only during an emergency or hurry during testing
        #make sure front is [+,+,+,+] and so on using the .ino file (for consistency across all codes)
        self.max_steer_pwm = 127

        #to change sign of value mapped to an axis, but to keep the direction of motion same for that published value, change in the joy callback
        #to change the direction of motion for a published value (sign), check wheel-wise using which_wheel.py and change default_dir in the .ino file

        rospy.init_node("drive_arc")
        rospy.Subscriber("joy", Joy, self.joyCallback)
        rospy.Subscriber("enc_auto", Float32MultiArray, self.enc_callback)
        rospy.Subscriber("/motion", WheelRpm, self.autonomous_motion_callback, queue_size = 10)
        rospy.Subscriber("rot", Int8,self.rotinplace_callback, queue_size=10)

        self.pwm_pub = rospy.Publisher('motor_pwm', Int32MultiArray, queue_size = 10)
        self.state_pub = rospy.Publisher('state', Bool, queue_size = 10) # Boolean to switch between autonomous and normal
        self.control = ['joystick, autonomous']
#        rospy.Subscriber('motion', WheelRpm, self.autonomous_callback)  
        self.pwm_msg = Int32MultiArray()
        self.pwm_msg.layout = MultiArrayLayout()
        self.pwm_msg.layout.data_offset = 0
        self.pwm_msg.layout.dim = [ MultiArrayDimension() ]
        self.pwm_msg.layout.dim[0].size = self.pwm_msg.layout.dim[0].stride = len(self.pwm_msg.data)
        self.pwm_msg.layout.dim[0].label = 'write'


        self.modeupbtn = 7
        self.modednbtn = 6

        self.fb_axis = 1    #to move rover forward-back
        self.lr_axis = 2    #to move rover left-right
        self.forward_btn = 4    #to turn all wheels front
        self.parallel_btn = 1   #to turn all wheels 90deg right
        self.rotinplace_btn = 3
        self.autonomous_btn = 0

        self.steer_unlock_axis = 4
        self.steer_samedir_axis = 2
        self.steer_oppdir_axis = 3

        self.full_potential_unlock_axis = 5
        self.fl_wheel_axis = 1
        self.fr_wheel_axis = 3
        self.bl_wheel_axis = 0
        self.br_wheel_axis = 2
        

        self.drive_ctrl = [0,0] #drive fb and lr axes
        self.steering_ctrl_locked = [0,0,0] #gives configurations for steering (buttons)
        self.curve_opp_str = 0

        self.steer_islocked = True
        self.steering_ctrl_unlocked = [0,0]     #buttons
        self.steering_ctrl_pwm = [0,0]      #joy control for steering (axes)
        
        self.full_potential_islocked = True
        #add buttons if necessary
        self.full_potential_pwm = [0,0,0,0]     #individual wheel control
         
        self.mode = 0 # from 0 to 4

        self.prints_per_iter = 3
        self.print_ctrl = self.prints_per_iter

        self.steering_complete = True
        #self.d_arr = [25,35,50,75,110] #same as galileo drive multipliers
        self.d_arr = [35,70,110,140,180]
        self.s_arr = [self.max_steer_pwm for i in range(5)] #no modes in steering       
        self.enc_data = [0,0,0,0]
        self.initial_enc_data = [0,0,0,0]
        self.initial_value_received = False
        self.kp_steer = 30
        self.qsize = 3
        self.vel_prev = queue.Queue(self.qsize) #to implement ledc type control
        self.omega_prev = queue.Queue(self.qsize) #to implement ledc type control
        self.start_time = time.time()
        self.time_thresh = 10
        self.error_thresh = 2.0  #degree

        self.rotinplace = False
        #autonomous params below
        self.crab_rotate = False
        self.autonomous_vel = 0
        self.autonomous_omega = 0
        self.rotin = 0
        self.state = False
        self.state_init = [False, False, False]

    def enc_callback(self,msg):
#        if(self.initial_value_received == False):
#            self.enc_data[0] = (msg.data)[0]
#            self.enc_data[1] = (msg.data)[3]
#            self.enc_data[2] = (msg.data)[1]
#            self.enc_data[3] = (msg.data)[5]
#            self.initial_enc_data[0] = (msg.data)[0]
#            self.initial_enc_data[1] = (msg.data)[3]
#            self.initial_enc_data[2] = (msg.data)[1]
#            self.initial_enc_data[3] = (msg.data)[5]
#            self.initial_value_received = True
#        else:
            print(len(msg.data))
            self.enc_data[0] = (msg.data)[1]    #front left
            self.enc_data[1] = -(msg.data)[4]   #front right
            self.enc_data[2] = (msg.data)[0]   #back left
            self.enc_data[3] = (msg.data)[5]    #back right
    
    def rotinplace_callback(self, msg):
        self.rotin = msg.data
        print(self.rotin)

    def autonomous_motion_callback(self,msg):
        if(self.state == True): #process only when in autonomous mode
            self.autonomous_vel = msg.vel #to offset to PWM
            self.autonomous_omega = msg.omega  #to offset to PWM
            self.crab_rotate = msg.hb #hb contains the crab rotation boolean
        

    def joyCallback(self, msg):
        if(self.state == False): #i.e. rover is in manual mode

        
            if(msg.buttons[self.modeupbtn] == 1):
                if(self.mode < 4):
                    self.mode = self.mode + 1
            if(msg.buttons[self.modednbtn] == 1):
                if(self.mode >0):
                    self.mode = self.mode - 1
                        
            if (self.steer_islocked == True and self.full_potential_islocked == True):
                self.steering_ctrl_locked = [msg.buttons[self.forward_btn], msg.buttons[self.parallel_btn], msg.buttons[self.rotinplace_btn]]
                self.drive_ctrl = [msg.axes[self.fb_axis], -msg.axes[self.lr_axis]]  #to change sign of value mapped to an axis, but to keep the direction of motion same for that published value, change here
                self.curve_opp_str = msg.axes[3]
                
            elif (self.steer_islocked == False and self.full_potential_islocked == True):
                self.steering_ctrl_unlocked = [msg.buttons[self.forward_btn],msg.buttons[self.parallel_btn]]    #for relative 45 (or any function buttons should perform)
                self.steering_ctrl_pwm = [msg.axes[self.steer_samedir_axis], msg.axes[self.steer_oppdir_axis]]  #for pwm to all motors

            elif (self.steer_islocked == True and self.full_potential_islocked == False):   #steer is locked, but full potential is unlocked
                #Add functionality for buttons in this state if needed
                self.full_potential_pwm = [msg.axes[self.fl_wheel_axis], msg.axes[self.fr_wheel_axis], msg.axes[self.bl_wheel_axis], msg.axes[self.br_wheel_axis]]

            #it should not enter else only

            if (msg.axes[self.steer_unlock_axis] == -1.0):  #Lock full potential when steering pwm is being toggled
                self.steer_islocked = not self.steer_islocked
                self.full_potential_islocked = True

            elif (msg.axes[self.full_potential_unlock_axis] == -1.0):   #Lock steering pwm when indiv control is being toggled
                self.full_potential_islocked = not self.full_potential_islocked
                self.steer_islocked = True

        if(msg.buttons[self.autonomous_btn] == 1):
            # to switch between manual and autonomous mode
            self.state = not self.state

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.main()
            rate.sleep()
            self.pwm_pub.publish(self.pwm_msg)

    def main(self):
        print(self.steering_ctrl_locked)
        
        if self.rotin!=0:
            print(f"+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++{self.rotin}+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
        if self.state==True:
            print(f"auto vel is {self.autonomous_vel}")
            print(f"auto omega is {self.autonomous_omega}")
            
        self.autonomous_control()
        self.steering()
        self.drive()
        self.print_ctrl = (self.print_ctrl+1) % self.prints_per_iter

    def autonomous_control(self):
        if(self.state == True): #rover is in autonomous mode, listen to commands on relevant topics
            if (self.print_ctrl == 0):    #printing only at certain intervals, to prevent the screen from being filed with data   #print_ctrl is being incremented in main() every time
                print("Rover in Autonomous Mode, Press 'A' to return to manual mode")
            
            if (self.rotin == 1 and self.state_init[0] == False):
                self.steering_ctrl_locked[2] = 1
                self.steering_ctrl_locked[0] = 0
                self.state_init[0] = True
            
            elif (self.rotin == 2 and self.state_init[1] == False):
                self.state_init[0] = False
                self.steering_ctrl_locked[0] = 1
                self.steering_ctrl_locked[2] = 0
                self.state_init[1] = True
            
            elif self.rotin == 0:
                self.steering_ctrl_locked[0] = 0
                self.steering_ctrl_locked[2] = 0
                self.state_init = [False, False, False]
            else:
                self.steering_ctrl_locked[0] = 0
                self.steering_ctrl_locked[2] = 0




    def steering(self):

        if (self.steer_islocked == True and self.full_potential_islocked == True):

            if(self.steering_ctrl_locked[0] == 1):
                print()
                print("Rotating steering forward")
                print()
                self.steering_complete = False
                self.rotinplace = False
                self.start_time = time.time()
                self.steer([0,0,0,0],[0,0,0,0],1)  #initial_angle is not used, final_angle, mode = 1 for absolute

            elif(self.steering_ctrl_locked[1] == 1):
                print()
                print("Rotating steering perpendicular to rover")
                print()
                self.steering_complete = False
                self.rotinplace = False
                self.start_time = time.time()
                self.steer([0,0,0,0],[90,90,90,90],1) #initial_angle is not used, final angle, mode=1 for absolute 
            
            elif(self.steering_ctrl_locked[2] == 1):
                print()
                print("Rotating steering for in place rotation")
                print()
                # rotinplace must be made true only in manual mode
                
                self.rotinplace = True
                self.steering_complete = False
                self.start_time = time.time()
                self.steer([0,0,0,0],[55,-55,-55,55],1) #rotating in place, mode=1 for absolute

            elif (abs(self.curve_opp_str) > 0.2):
                self.steering_complete = False
                self.rotinplace = False
                self.start_time = time.time()
                temp = int(self.s_arr[self.mode] * self.curve_opp_str)
                self.pwm_msg.data = [0,0,0,0,temp*self.init_dir[4],temp*self.init_dir[5],-temp*self.init_dir[6],-temp*self.init_dir[7]]
                print("Encoder angles:-", self.enc_data, end = "       ") 
                print("Mode =", self.mode, end = "      ")
                print("Curving with steering")

            else:
                self.pwm_msg.data = [0,0,0,0,0,0,0,0]
        
        elif (self.steer_islocked == False and self.full_potential_islocked == True):
            enc_data_new = copy.deepcopy(self.enc_data) # to create a deep copy of enc_data array, not a pointer equivalence. Is being used as the inital angle
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

            elif (self.steering_ctrl_pwm[0]!=0 and abs(self.steering_ctrl_pwm[1]) < 0.2):     #edit here to give operator threshold
                self.steering_complete = False
                self.rotinplace = False
                self.start_time = time.time()
                temp = -int(self.s_arr[self.mode] * self.steering_ctrl_pwm[0])
                self.pwm_msg.data = [0,0,0,0,temp*self.init_dir[4],temp*self.init_dir[5],temp*self.init_dir[6],temp*self.init_dir[7]]
                print("Encoder angles:-", self.enc_data, end = "       ") 
                print("Mode =", self.mode, end = "      ")
                print("All wheels -> same direction.")

            elif (self.steering_ctrl_pwm[1] != 0 and abs(self.steering_ctrl_pwm[0]) < 0.2):
                print()
                print("Moving with rot in place velocities")
                print()
                temp = int(self.max_steer_pwm*self.steering_ctrl_pwm[1])
                self.pwm_msg.data = [0,0,0,0,temp*self.init_dir[4],-temp*self.init_dir[5],-temp*self.init_dir[6],temp*self.init_dir[7]]
                print("Rotating in place with velocity =",temp)
                print("Enc_angles:- ", self.enc_data)

            else:
                self.pwm_msg.data = [0,0,0,0,0,0,0,0]
                if (self.print_ctrl == 0):    #printing only at certain intervals, to prevent the screen from being filed with data
                    print("Steering is unlocked, lock it to perform drive.")

        elif (self.steer_islocked == True and self.full_potential_islocked == False):

            if (self.full_potential_pwm[0] != 0 and abs(self.full_potential_pwm[2]) < 0.2):   #front left wheel
                temp = int(self.s_arr[self.mode] * self.full_potential_pwm[0])
                self.pwm_msg.data = [0,0,0,0,temp*self.init_dir[4],0,0,0]
                print("Encoder angles:-", self.enc_data, end = "       ")
                print("Mode =", self.mode, end = "      ")
                print("Moving front left wheel.")

            elif (self.full_potential_pwm[1] != 0 and abs(self.full_potential_pwm[3]) < 0.2):     #front right wheel
                temp = int(self.s_arr[self.mode] * self.full_potential_pwm[1])
                self.pwm_msg.data = [0,0,0,0,0,temp*self.init_dir[5],0,0]
                print("Encoder angles:-", self.enc_data, end = "       ")
                print("Mode =", self.mode, end = "      ")
                print("Moving front right wheel.")
                
            elif (self.full_potential_pwm[2] != 0 and abs(self.full_potential_pwm[0]) < 0.2):     #back left wheel
                temp = int(self.s_arr[self.mode] * self.full_potential_pwm[2])
                self.pwm_msg.data = [0,0,0,0,0,0,-temp*self.init_dir[6],0]
                print("Encoder angles:-", self.enc_data, end = "       ")
                print("Mode =", self.mode, end = "      ")
                print("Moving back left wheel.")

            elif (self.full_potential_pwm[3] != 0 and abs(self.full_potential_pwm[1]) < 0.2):     #back right wheel
                temp = int(self.s_arr[self.mode] * self.full_potential_pwm[3])
                self.pwm_msg.data = [0,0,0,0,0,0,0,-temp*self.init_dir[7]]
                print("Encoder angles:-", self.enc_data, end = "       ")
                print("Mode =", self.mode, end = "      ")
                print("Moving back right wheel.")

            else:
                self.pwm_msg.data = [0,0,0,0,0,0,0,0]
                if (self.print_ctrl == 0):    #printing only at certain intervals, to prevent the screen from being filed with data
                    print("Individual steering control mode unlocked, lock it to perform drive.")

        self.steering_complete = True
        self.start_time = time.time()

    def drive(self):
        if(self.steering_complete == True and self.steer_islocked == True and self.full_potential_islocked == True):
             
            if (self.rotinplace == True):
                if(self.state == False):
                    vel = self.d_arr[self.mode] * self.drive_ctrl[1]
                    self.pwm_msg.data = [int(vel)*self.init_dir[0], -int(vel)*self.init_dir[1],int(vel)*self.init_dir[2],-int(vel)*self.init_dir[3], 0,0,0,0]
                    if (self.print_ctrl == 0):    #printing only at certain intervals, to prevent the screen from being filed with data   #print_ctrl is being incremented in main() every time
                        print("Rotation speed =", int(vel))
                else:
                    vel = self.autonomous_omega
                    self.pwm_msg.data = [int(vel)*self.init_dir[0], -int(vel)*self.init_dir[1],int(vel)*self.init_dir[2], -int(vel)*self.init_dir[3], 0,0,0,0]
                    if (self.print_ctrl == 0):    #printing only at certain intervals, to prevent the screen from being filed with data   #print_ctrl is being incremented in main() every time
                        print("Rotation speed =", int(vel))
            else:
                velocity = 0
                omega = 0
                if (self.state == False):
                    omega = self.d_arr[self.mode] * self.drive_ctrl[1]
                    velocity = self.d_arr[self.mode] * self.drive_ctrl[0]

                else:
                    velocity = self.autonomous_vel
                    omega = self.autonomous_omega

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

                #self.pwm_msg.data = [int(avg_velocity-avg_omega), int(avg_velocity+avg_omega), int(avg_velocity-avg_omega), int(avg_velocity+avg_omega), 0,0,0,0]
                self.pwm_msg.data[0] = int(avg_velocity+avg_omega)*self.init_dir[0]
                self.pwm_msg.data[1] = int(avg_velocity-avg_omega)*self.init_dir[1]
                self.pwm_msg.data[2] = int(avg_velocity+avg_omega)*self.init_dir[2]
                self.pwm_msg.data[3] = int(avg_velocity-avg_omega)*self.init_dir[3]
            #standard code

            self.pwm_msg.layout = MultiArrayLayout()
            self.pwm_msg.layout.data_offset = 0
            self.pwm_msg.layout.dim = [ MultiArrayDimension() ]
            self.pwm_msg.layout.dim[0].size = self.pwm_msg.layout.dim[0].stride = len(self.pwm_msg.data)
            self.pwm_msg.layout.dim[0].label = 'write'
        else:
            pass


    def steer(self, initial_angles, final_angles, mode):
        pwm = [0,0,0,0]
        pwm_temp = [0,0,0,0]
        if(mode == 0):      # relative (encoder will go <final_angle> away from its initial position)
            while( (abs(self.enc_data[0] - initial_angles[0]) < abs(final_angles[0])-self.error_thresh or abs(self.enc_data[1] - initial_angles[1]) < abs(final_angles[1])-self.error_thresh or abs(self.enc_data[2] - initial_angles[2]) < abs(final_angles[2])-self.error_thresh or abs(self.enc_data[3] - initial_angles[3]) < abs(final_angles[3])-self.error_thresh)  and time.time() - self.start_time <= self.time_thresh): #time_thresh = 10s   #error_thresh = 5 degrees

                if(int(time.time() - self.start_time) * 10 % 2 == 0):  #printing only at certain intervals, to prevent the screen from being filed with data
                    print("Enc_data:- ", self.enc_data, end = "     ")
                    print("Final angles:- ", list(map(add, initial_angles, final_angles)))
                
                for i in range(4):
                    if (abs(self.enc_data[i] - initial_angles[i]) < abs(final_angles[i])-self.error_thresh):
                        pwm_temp[i] = int(self.kp_steer*(final_angles[i]-(self.enc_data[i]-initial_angles[i])))
                        if (pwm_temp[i]>=0):
                            pwm[i] = min(self.max_steer_pwm,pwm_temp[i])
                        else:
                            pwm[i] = max(-self.max_steer_pwm,pwm_temp[i])
                    else:
                        pwm[i] = 0

                self.pwm_msg.data = [0,0,0,0,pwm[0]*self.init_dir[4],pwm[1]*self.init_dir[5],pwm[2]*self.init_dir[6],pwm[3]*self.init_dir[7]]

                #standard code
                rate = rospy.Rate(10)
                rate.sleep()
                self.pwm_pub.publish(self.pwm_msg)
                                

        elif(mode == 1):        # absolute (encoder will become equal to the final angle specified)
            while( (abs(self.enc_data[0] - final_angles[0]) > self.error_thresh or abs(self.enc_data[1] - final_angles[1]) > self.error_thresh or abs(self.enc_data[2] - final_angles[2]) > self.error_thresh or abs(self.enc_data[3] - final_angles[3]) > self.error_thresh) and time.time() - self.start_time <= self.time_thresh):   #error_thresh = 5 degrees

                if(int(time.time() - self.start_time) * 10 % 2 == 0):   #printing only at certain intervals, to prevent the screen from being filed with data
                    print("Enc_data:- ", self.enc_data, end = "     ")
                    print("Final angles:- ", final_angles)

                for i in range(4):
                    if (abs(self.enc_data[i] - final_angles[i]) > self.error_thresh):
                        pwm_temp[i] = int(self.kp_steer*(final_angles[i]-self.enc_data[i]))
                        if (pwm_temp[i]>=0):
                            pwm[i] = min(self.max_steer_pwm,pwm_temp[i])
                        else:
                            pwm[i] = max(-self.max_steer_pwm,pwm_temp[i])
                    else:
                        pwm[i] = 0

                print(pwm)
                self.pwm_msg.data = [0,0,0,0,pwm[0]*self.init_dir[4],pwm[1]*self.init_dir[5],pwm[2]*self.init_dir[6],pwm[3]*self.init_dir[7]]

                rate = rospy.Rate(10)
                rate.sleep()
                self.pwm_pub.publish(self.pwm_msg)
        
        print()
        print("***Steering Complete***")
        print()
    
if __name__ == '__main__':
    run = Drive()
    run.spin()

