#!/usr/bin/env python3
import rospy
import copy
from sensor_msgs.msg import Joy
import time
from std_msgs.msg import Int8
from std_msgs.msg import Float32
from std_msgs.msg import Int32MultiArray, MultiArrayLayout, MultiArrayDimension, Float32MultiArray
import queue
from operator import add

class Drive:
    def __init__(self):
        rospy.init_node("drive_arc")
        rospy.Subscriber("joy", Joy, self.joyCallback)
        rospy.Subscriber("enc_auto", Float32MultiArray, self.enc_callback)
        self.pwm_pub = rospy.Publisher('motor_pwm', Int32MultiArray, queue_size = 10)
        self.control = ['joystick, autonomous']
        
        self.pwm_msg = Int32MultiArray()
        self.pwm_msg.layout = MultiArrayLayout()
        self.pwm_msg.layout.data_offset = 0
        self.pwm_msg.layout.dim = [ MultiArrayDimension() ]
        self.pwm_msg.layout.dim[0].size = self.pwm_msg.layout.dim[0].stride = len(self.pwm_msg.data)
        self.pwm_msg.layout.dim[0].label = 'write'


        self.modeupbtn = 7
        self.modednbtn = 6

        self.fb_axis = 1    #to move rover forward-back
        self.car_axis = 2    #to move rover left-right
        self.front_clock = 1    #buttons
        self.front_anti = 3
        self.car_ctrl = [0,0] #drive fb and steering car axes
        self.fixed_car_angles = [0,0] #gives configurations for steering (buttons)
        self.button_car_angle = 30

        self.encoder_unlock_axis = 4
        self.forward_btn = 4    #to turn all wheels front
        self.parallel_btn = 1   #to turn all wheels 90deg right
        self.rotinplace_btn = 3
        self.encoder_cup = True
        self.drive_ctrl = [0,0]
        self.encoder_working_buttons = [0,0,0]     #buttons

        self.indiv_steer_unlock_axis = 5
        self.fl_wheel_axis = 1
        self.fr_wheel_axis = 3
        self.bl_wheel_axis = 0
        self.br_wheel_axis = 2
        self.indiv_locked = True
        #add buttons if necessary
        self.indiv_steer_pwm = [0,0,0,0]     #individual wheel control
        
        self.mode = 0 # from 0 to 4

        self.prints_per_iter = 3
        self.print_ctrl = self.prints_per_iter

        self.steering_complete = True
        self.d_arr = [25,35,50,75,110] #same as galileo drive multipliers 
        self.s_arr = [255 for i in range(5)] #no modes in steering       
        self.enc_data = [0,0,0,0]
        self.kp_steer = 30
        self.qsize = 5
        self.vel_prev = queue.Queue(self.qsize) #to implement ledc type control
        self.diff_vel_prev = queue.Queue(self.qsize) #to implement ledc type control
        self.omega_prev = queue.Queue(self.qsize) #to implement ledc type control
        self.start_time = time.time()
        self.time_thresh = 10
        self.error_thresh = 1   #degree
        self.initial_enc_data = [0,0,0,0]

        self.rotinplace = False

        self.initial_value_received = False

    def enc_callback(self,msg):
        if(self.initial_value_received == False):
            self.enc_data[0] = (msg.data)[0]
            self.enc_data[1] = (msg.data)[3]
            self.enc_data[2] = (msg.data)[2]
            self.enc_data[3] = (msg.data)[5]
            self.initial_enc_data[0] = (msg.data)[0]
            self.initial_enc_data[1] = (msg.data)[3]
            self.initial_enc_data[2] = (msg.data)[2]
            self.initial_enc_data[3] = (msg.data)[5]
            self.initial_value_received = True
        else:
            self.enc_data[0] = (msg.data)[0] - self.initial_enc_data[0]
            self.enc_data[1] = (msg.data)[3] - self.initial_enc_data[1]
            self.enc_data[2] = (msg.data)[2] - self.initial_enc_data[2]
            self.enc_data[3] = (msg.data)[5] - self.initial_enc_data[3]

    def joyCallback(self, msg):
        
        if(msg.buttons[self.modeupbtn] == 1):
            if(self.mode < 4):
                self.mode = self.mode + 1
        if(msg.buttons[self.modednbtn] == 1):
            if(self.mode >0):
                self.mode = self.mode - 1
                    
        if (self.encoder_cup == True and self.indiv_locked == True):
            self.fixed_car_angles = [msg.buttons[self.front_clock], msg.buttons[self.front_anti]]
            self.car_ctrl = [msg.axes[self.fb_axis], msg.axes[self.car_axis]]
            
        elif (self.encoder_cup == False and self.indiv_locked == True):
            self.encoder_working_buttons = [msg.buttons[self.forward_btn], msg.buttons[self.parallel_btn], msg.buttons[self.rotinplace_btn]]
            self.drive_ctrl = [msg.axes[1], msg.axes[2]]

        elif (self.encoder_cup == True and self.indiv_locked == False):   #steer is locked, but full potential is unlocked
            #Add functionality for buttons in this state if needed
            self.indiv_steer_pwm = [msg.axes[self.fl_wheel_axis], msg.axes[self.fr_wheel_axis], msg.axes[self.bl_wheel_axis], msg.axes[self.br_wheel_axis]]

        #it should not enter else only

        if (msg.axes[self.encoder_unlock_axis] == -1.0):  #Lock full potential when steering pwm is being toggled
            self.encoder_cup = not self.encoder_cup
            self.indiv_locked = True

        elif (msg.axes[self.indiv_steer_unlock_axis] == -1.0):   #Lock steering pwm when indiv control is being toggled
            self.indiv_locked = not self.indiv_locked
            self.encoder_cup = True
 
    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.main()
            rate.sleep()
            self.pwm_pub.publish(self.pwm_msg)

    def main(self):
        self.steering()
        self.drive()
        self.print_ctrl = (self.print_ctrl+1) % self.prints_per_iter

    def steering(self):

        if (self.encoder_cup == True and self.indiv_locked == True):
            
            enc_data_new = copy.deepcopy(self.enc_data) # to create a deep copy of enc_data array, not a pointer equivalence. Is being used as the inital angle
            self.rotinplace = False
            self.steering_complete = False

            if (self.fixed_car_angles[0] == 1):
                print()
                print("Front two wheels by", self.button_car_angle ,"deg clockwise")
                print()
                self.start_time = time.time()
                self.steer(enc_data_new,[self.button_car_angle, self.button_car_angle, 0, 0],0)

            elif (self.fixed_car_angles[1] == 1):
                print()
                print("Front two wheels by", self.button_car_angle ,"deg anti-clockwise")
                print()
                self.start_time = time.time()
                self.steer(enc_data_new,[-self.button_car_angle, -self.button_car_angle, 0, 0],0)

            elif (abs(self.car_ctrl[1])>0.2):
                self.start_time = time.time()
                temp = int(self.s_arr[self.mode] * self.car_ctrl[1])
                self.pwm_msg.data = [0,0,0,0,-temp,-temp,0,0]
                print("Encoder angles:-", [self.enc_data[0], self.enc_data[1]], end = "       ")
                print("Moving front two wheels.")

            else:
                self.pwm_msg.data = [0,0,0,0,0,0,0,0]

        elif (self.encoder_cup == False and self.indiv_locked == True):

            self.steering_complete = False

            if(self.encoder_working_buttons[0] == 1):
                print()
                print("Rotating steering forward")
                print()
                self.rotinplace = False
                self.start_time = time.time()
                self.steer([0,0,0,0],[0,0,0,0],1)  #initial_angle is not used, final_angle, mode = 1 for absolute

            elif(self.encoder_working_buttons[1] == 1):
                print()
                print("Rotating steering perpendicular to rover")
                print()
                self.rotinplace = False
                self.start_time = time.time()
                self.steer([0,0,0,0],[90,90,90,90],1) #initial_angle is not used, final angle, mode=1 for absolute 

            elif(self.encoder_working_buttons[2] == 1):
                print()
                print("Rotating steering for in place rotation")
                print()
                self.rotinplace = True
                self.steering_complete = False
                self.start_time = time.time()
                self.steer([0,0,0,0],[45,-45,-45,45],1) #rotating in place, mode=1 for absolute
            else:
                self.pwm_msg.data = [0,0,0,0,0,0,0,0]
                print("Encoder and differential drive mode.")
                print()

        elif (self.encoder_cup == True and self.indiv_locked == False):

            if (self.indiv_steer_pwm[0] != 0 and abs(self.indiv_steer_pwm[2]) < 0.2):   #front left wheel
                temp = int(self.s_arr[self.mode] * self.indiv_steer_pwm[0])
                self.pwm_msg.data = [0,0,0,0,temp,0,0,0]
                print("Encoder angles:-", self.enc_data, end = "       ")
                print("Mode =", self.mode, end = "      ")
                print("Moving front left wheel.")

            elif (self.indiv_steer_pwm[1] != 0 and abs(self.indiv_steer_pwm[3]) < 0.2):     #front right wheel
                temp = int(self.s_arr[self.mode] * self.indiv_steer_pwm[1])
                self.pwm_msg.data = [0,0,0,0,0,temp,0,0]
                print("Encoder angles:-", self.enc_data, end = "       ")
                print("Mode =", self.mode, end = "      ")
                print("Moving front right wheel.")
                
            elif (self.indiv_steer_pwm[2] != 0 and abs(self.indiv_steer_pwm[0]) < 0.2):     #back left wheel
                temp = int(self.s_arr[self.mode] * self.indiv_steer_pwm[2])
                self.pwm_msg.data = [0,0,0,0,0,0,temp,0]
                print("Encoder angles:-", self.enc_data, end = "       ")
                print("Mode =", self.mode, end = "      ")
                print("Moving back left wheel.")

            elif (self.indiv_steer_pwm[3] != 0 and abs(self.indiv_steer_pwm[1]) < 0.2):     #back right wheel
                temp = int(self.s_arr[self.mode] * self.indiv_steer_pwm[3])
                self.pwm_msg.data = [0,0,0,0,0,0,0,temp]
                print("Encoder angles:-", self.enc_data, end = "       ")
                print("Mode =", self.mode, end = "      ")
                print("Moving back right wheel.")

            else:
                self.pwm_msg.data = [0,0,0,0,0,0,0,0]
                if (self.print_ctrl == 0):    #printing only at certain intervals, to prevent the screen from being filed with data
                    print("Individual steering control mode.")
                    print()

        self.steering_complete = True
        self.start_time = time.time()

    def drive(self):
        if(self.indiv_locked == True and self.encoder_cup == True):
             
            velocity = self.d_arr[self.mode] * self.car_ctrl[0]

            avg_velocity = 0
            if(self.vel_prev.full()):
                #calculating average of values in queues
                for j in self.vel_prev.queue:
                    avg_velocity = avg_velocity+j
                avg_velocity = avg_velocity / self.qsize

                #removing a value from the queues
                self.vel_prev.get() 

            #adding a value to each of the queues
            self.vel_prev.put(velocity, True, 2)

            print("Drive Velocity:", (avg_velocity//0.001)/1000, end = "    ")   # the math part is to only print upto 2 decimals
            print("Mode: ", self.mode)
            print()

            #self.pwm_msg.data = [int(avg_velocity), int(avg_velocity), int(avg_velocity), int(avg_velocity), 0,0,0,0]
            self.pwm_msg.data[0] = int(avg_velocity)
            self.pwm_msg.data[1] = int(avg_velocity)
            self.pwm_msg.data[2] = int(avg_velocity)
            self.pwm_msg.data[3] = int(avg_velocity)


            #standard code

            self.pwm_msg.layout = MultiArrayLayout()
            self.pwm_msg.layout.data_offset = 0
            self.pwm_msg.layout.dim = [ MultiArrayDimension() ]
            self.pwm_msg.layout.dim[0].size = self.pwm_msg.layout.dim[0].stride = len(self.pwm_msg.data)
            self.pwm_msg.layout.dim[0].label = 'write'

        elif (self.encoder_cup == False and self.indiv_locked == True):
            if (self.rotinplace == True):
                vel = self.d_arr[self.mode] * self.drive_ctrl[1]
                self.pwm_msg.data = [-int(vel), int(vel),- int(vel), int(vel), 0,0,0,0]
                print("Rotation speed =", int(vel))
            else:
                velocity = self.d_arr[self.mode] * self.drive_ctrl[0]
                omega = self.d_arr[self.mode] * self.drive_ctrl[1]

                avg_velocity, avg_omega = 0, 0
                if(self.diff_vel_prev.full() and self.omega_prev.full()):
                    #calculating average of values in queues
                    for i in self.omega_prev.queue:
                        avg_omega = avg_omega + i
                    for j in self.diff_vel_prev.queue:
                        avg_velocity = avg_velocity+j
                    avg_velocity = avg_velocity / self.qsize
                    avg_omega = avg_omega / self.qsize

                    #removing a value from the queues
                    self.diff_vel_prev.get()
                    self.omega_prev.get()

                #adding a value to each of the queues
                self.diff_vel_prev.put(velocity, True, 2)
                self.omega_prev.put(omega, True, 2)

                print("Velocity:", (avg_velocity//0.001)/1000, end = "     ")   # the math part is to only print upto 2 decimals
                print("Omega: ", (avg_omega//0.001)/1000, end = "     ")
                print("Mode: ", self.mode)
                print()

                self.pwm_msg.data = [int(avg_velocity-avg_omega), int(avg_velocity+avg_omega), int(avg_velocity-avg_omega), int(avg_velocity+avg_omega), 0,0,0,0]
               






            #standard code

            self.pwm_msg.layout = MultiArrayLayout()
            self.pwm_msg.layout.data_offset = 0
            self.pwm_msg.layout.dim = [ MultiArrayDimension() ]
            self.pwm_msg.layout.dim[0].size = self.pwm_msg.layout.dim[0].stride = len(self.pwm_msg.data)
            self.pwm_msg.layout.dim[0].label = 'write'

        else:
            pass


    def steer(self, initial_angles, final_angles, mode):

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
                        pwm[i] = int(self.kp_steer*(final_angles[i]-self.enc_data[i]))
                    else:
                        pwm[i] = 0

                self.pwm_msg.data = [0,0,0,0,pwm[0],pwm[1],pwm[2],pwm[3]]

                rate = rospy.Rate(10)
                rate.sleep()
                self.pwm_pub.publish(self.pwm_msg)
        
        print()
        print("***Steering Complete***")
        print()
    
if __name__ == '__main__':
    run = Drive()
    run.spin()

