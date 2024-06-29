#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
import time
from std_msgs.msg import Int8
from std_msgs.msg import Float32
from std_msgs.msg import Int32MultiArray, MultiArrayLayout, MultiArrayDimension
import queue

class Drive:
    def __init__(self):
        rospy.init_node("drive_arc")
        rospy.Subscriber("joy", Joy, self.joyCallback)
        rospy.Subscriber("enc_auto", Float32, self.enc_callback)
        self.pwm_pub = rospy.Publisher('motor_pwm', Int32MultiArray, queue_size = 10)
        self.control = ['joystick, autonomous']
        self.pwm_msg = Int32MultiArray()
        

        self.forward_btn = 4
        self.parallel_btn = 1
        self.rotinplace_btn = 3
        self.modeupbtn = 7
        self.modednbtn = 6
        self.steer_absrel_btn = 0
        
        self.steering_ctrl = [0,0,0] #gives modes for steering
        self.drive_ctrl = [0,0] #drive fb and lr axes
        self.fb_axis = 1 #forward-back
        self.lr_axis = 0 #left-right
        self.mode = 0 # from 0 to 4
        self.steering_complete = True
        self.s_arr = [25,35,50,75,110] #same as galileo drive multipliers
        self.enc_data = 0
        self.kp_steer = 1
        self.qsize = 5
        self.vel_prev = queue.Queue(self.qsize) #to implement ledc type control
        self.omega_prev = queue.Queue(self.qsize) #to implement ledc type control
        self.start_time = time.time()
        self.time_thresh = 10
        self.error_thresh = 2
        self.absrel_ctrl = 1 #starts with absolute mode for 90 deg

        self.rotinplace = False
    def enc_callback(self,msg):
        self.enc_data = msg.data

    def joyCallback(self, msg):
        self.steering_ctrl = [msg.buttons[self.forward_btn], msg.buttons[self.parallel_btn], msg.buttons[self.rotinplace_btn]]
        if(msg.buttons[self.modeupbtn] == 1):
            if(self.mode < 4):
                self.mode = self.mode + 1
        if(msg.buttons[self.modednbtn] == 1):
            if(self.mode >0):
                self.mode = self.mode - 1
        self.drive_ctrl = [msg.axes[self.fb_axis], msg.axes[self.lr_axis]]
        if(msg.buttons[self.steer_absrel_btn] == 1):
            self.absrel_ctrl = 1-self.absrel_ctrl
    
    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.main()
            rate.sleep()
            self.pwm_pub.publish(self.pwm_msg)


    def main(self):
        self.steering()
        self.drive()

    def steering(self):
        if(self.steering_ctrl[0] == 1):
            print("Rotating steering forward")
            self.steering_complete = False
            self.rotinplace = False
            #loop here with encoder feedback
            self.start_time = time.time()
            if(self.absrel_ctrl == 0):
                print("90 degree relative turn")
                self.steer(self.enc_data,90,0)  #initial_angle = enc_data, final_angle, mode = 0 for relative
            else:
                print("90 degree absolute turn")
                self.steer(0,90,1)


        elif(self.steering_ctrl[1] == 1):
            print("Rotating steering horizontally (parallely)")
            self.steering_complete = False
            self.rotinplace = False
            #loop here with encoder feedback
            self.start_time = time.time()
            self.steer(0,0,1) #initial angle, final angle, mode=1 for absolute 
        
        elif(self.steering_ctrl[2] == 1):
            print("Rotating steering for in place rotation")
            self.rotinplace = True
            self.steering_complete = False
            #loop here with encoder feedback and timing
            self.start_time = time.time()
            
        self.steering_complete = True
        self.start_time = time.time()
        print("***Steering Complete****")

    def drive(self):
        if(self.steering_complete == True):
            velocity =self.s_arr[self.mode]*self.drive_ctrl[0]
            omega =self.s_arr[self.mode]*self.drive_ctrl[1]
            avg_velocity, avg_omega = 0, 0
            if(self.vel_prev.full() and self.omega_prev.full()):
                for i in self.omega_prev.queue:
                    avg_omega = avg_omega + i
                for j in self.vel_prev.queue:
                    avg_velocity = avg_velocity+j
                avg_velocity = avg_velocity / self.qsize
                avg_omega = avg_omega / self.qsize

                ret_vel, ret_omega = self.vel_prev.get(), self.omega_prev.get()
                print("Omega, Vel in Queue: ", ret_vel , ret_omega, avg_velocity, avg_omega)
            offset = 0
            print("Velocity:", avg_velocity)
            print("Omega: ", avg_omega )
            print("Mode: ", self.mode)
            self.pwm_msg.data = [int(avg_velocity+avg_omega), int(avg_velocity+avg_omega), int(avg_velocity-avg_omega), int(avg_velocity-avg_omega), 0,0,0,0]

            self.pwm_msg.layout = MultiArrayLayout()
            self.pwm_msg.layout.data_offset = 0

            self.pwm_msg.layout.dim = [ MultiArrayDimension() ]
            self.pwm_msg.layout.dim[0].size = self.pwm_msg.layout.dim[0].stride = len(self.pwm_msg.data)
            self.pwm_msg.layout.dim[0].label = 'write'
            self.vel_prev.put(velocity, True, 2)
            self.omega_prev.put(omega, True, 2)


    def steer(self,initial_angle, final_angle, mode):
        print("steering called", self.enc_data, initial_angle)
        if(mode == 0):
            while(abs(self.enc_data - initial_angle) < final_angle-self.error_thresh and time.time() - self.start_time <= self.time_thresh):
                if(int(time.time() - self.start_time * 10) % 2 == 0):
                    print("Executing steering", self.enc_data)
                self.pwm_msg.data = [0,0,0,0,int(-self.kp_steer*(final_angle-self.enc_data+initial_angle)),int(-self.kp_steer*(final_angle-(self.enc_data-initial_angle))),int(-self.kp_steer*(final_angle - (self.enc_data-initial_angle))),int(-self.kp_steer*(final_angle - (self.enc_data-initial_angle)))]

                self.pwm_msg.layout = MultiArrayLayout()
                self.pwm_msg.layout.data_offset = 0

                self.pwm_msg.layout.dim = [ MultiArrayDimension() ]
                self.pwm_msg.layout.dim[0].size = self.pwm_msg.layout.dim[0].stride = len(self.pwm_msg.data)
                self.pwm_msg.layout.dim[0].label = 'write'
                rate = rospy.Rate(10)
                rate.sleep()
                self.pwm_pub.publish(self.pwm_msg)
        elif(mode == 1):
            enc_data_init = self.enc_data
            while(abs(self.enc_data - initial_angle-final_angle) > self.error_thresh and time.time() - self.start_time <= self.time_thresh):
                if(int(time.time() - self.start_time * 10) % 2 == 0):
                    print("Executing steering", self.enc_data)
                self.pwm_msg.data = [0,0,0,0,int(-self.kp_steer*(final_angle-self.enc_data+initial_angle)),int(-self.kp_steer*(final_angle-(self.enc_data-initial_angle))),int(-self.kp_steer*(final_angle - (self.enc_data-initial_angle))),int(-self.kp_steer*(final_angle - (self.enc_data-initial_angle)))]

                self.pwm_msg.layout = MultiArrayLayout()
                self.pwm_msg.layout.data_offset = 0

                self.pwm_msg.layout.dim = [ MultiArrayDimension() ]
                self.pwm_msg.layout.dim[0].size = self.pwm_msg.layout.dim[0].stride = len(self.pwm_msg.data)
                self.pwm_msg.layout.dim[0].label = 'write'
                rate = rospy.Rate(10)
                rate.sleep()
                self.pwm_pub.publish(self.pwm_msg)
        
        
        else:
            print("In Rotation")

    
if __name__ == '__main__':
    run = Drive()
    run.spin()
