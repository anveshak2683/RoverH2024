#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class EKFSimplified:
    def __init__(self):
        rospy.init_node('ekf_simplified')
        rospy.loginfo("EKF Simplified Node Started")

        # Initialize state [x, y, orientation]
        self.state = np.array([0.0, 0.0, 0.0])
        self.covariance = np.eye(3) * 1000
        self.Q = np.diag([0.5, 0.5])  # Process noise
        self.R_imu = np.diag([0.001, 0.001])  # Placeholder for IMU data covariance
        self.R_lidar = np.diag([0.1, 0.1, 0.1])  # Lidar covariance for (x, y, orientation)

        self.initialized_imu = False
        self.initialized_lidar = False
        self.last_time = None
        self.delta_t=0.0
        self.linear_acceleration_x= 0.0
        self.linear_acceleration_y= 0.0
        self.omega_z=0.0
        self.measurement =np.array([0.0, 0.0, 0.0])

        self.vx=0.0  # Velocity in x direction
        self.vy=0.0  # Velocity in y direction
        self.threshold_v=0.20  #  This is the velocity where the ekf starts to predict (otherwise even when rover stops the velocity estimates may have some value)
        self.threshold_accel=1.5 # This is the acceleration where the ekf starts to predict

        # Publishers and Subscribers
        self.odom_pub = rospy.Publisher('/ekf_odometry', Odometry, queue_size=10)
        rospy.Subscriber('/robot/dlo/odom_node/odom', Odometry, self.lidar_callback)
        rospy.Subscriber("/zed2i/zed_node/imu/data", Imu, self.imu_callback)

    def imu_callback(self, msg):
        current_time = rospy.get_time()
        if self.last_time is None:
            self.last_time = current_time
            return

        self.delta_t = current_time - self.last_time
        self.last_time = current_time

        if not self.initialized_imu:
            # Assume initial orientation from IMU
            orientation = msg.orientation
            _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
            self.state[2] = yaw
            self.initialized_imu = True
            return

        # Extract yaw rate, and lienar acceleration
        self.omega_z = msg.angular_velocity.z
        self.linear_acceleration_x = msg.linear_acceleration.x
        self.linear_acceleration_y = msg.linear_acceleration.y

        #print("IMU")

        
        
        #self.publish_odometry()

    def lidar_callback(self, msg):
        if not self.initialized_lidar:
            position=msg.pose.pose.position
            self.state[0] = position.x
            self.state[1] = position.y
            self.initialized_lidar = True
            return
        #print("Lidar")

        # Measurement update
        self.measurement = np.array([msg.pose.pose.position.x,
                      msg.pose.pose.position.y,
                      euler_from_quaternion([msg.pose.pose.orientation.x,
                                             msg.pose.pose.orientation.y,
                                             msg.pose.pose.orientation.z,
                                             msg.pose.pose.orientation.w])[2]])

        

    def predict(self):
        # Convert acceleration to velocity

        #linear_acceleration_x = np.clip(linear_acceleration_x, -0.01, 0.01)
        #linear_acceleration_y = np.clip(linear_acceleration_y, -0.01, 0.01)

        # Compute the acceleration magnitude
        accel_magnitude = np.sqrt(self.linear_acceleration_x**2 + self.linear_acceleration_y**2)

        if accel_magnitude > self.threshold_accel:
            # Convert acceleration to velocity
            self.vx += self.linear_acceleration_x * self.delta_t
            self.vy += self.linear_acceleration_y * self.delta_t
            print("a_x:", self.linear_acceleration_x)
            print("a_y:", self.linear_acceleration_y)
            
    

        #if linear_acceleration_x > 0.02 and linear_acceleration_x < -0.02:
        #self.vx += linear_acceleration_x * delta_t
        #self.vy += linear_acceleration_y * delta_t

        
        
         # Calculate velocity magnitude in the direction of the current orientation
        v_magnitude = np.sqrt(self.vx**2 + self.vy**2)

        if v_magnitude < self.threshold_v:
            self.vx=0.0
            self.vy=0.0

        '''if v_magnitude < (self.threshold_v + 0.03) or v_magnitude > (self.threshold_v - 0.03):
            self.vx=0.0
            self.vy=0.0'''
        '''if v_magnitude < (self.threshold_v):  # Allow slightly more flexibility
            decay_factor = 0.97
            self.vx *= decay_factor
            self.vy *= decay_factor'''
        print("v:", v_magnitude)
        print("vx:", self.vx)
        print("vy:", self.vy)

        # State prediction 
        orientation = self.state[2]
        self.state[0] += self.vx * self.delta_t
        self.state[1] += self.vy * self.delta_t
        self.state[2] += self.omega_z * self.delta_t
        self.state[2] = (self.state[2] + np.pi) % (2 * np.pi) - np.pi  # Normalize orientation

        # Compute Jacobians
        Fp = np.array([
            [1, 0, -v_magnitude * np.sin(orientation) * self.delta_t],
            [0, 1, v_magnitude * np.cos(orientation) * self.delta_t],
            [0, 0, 1]
        ])

        Fu = np.array([
            [np.cos(orientation) * self.delta_t, 0],
            [np.sin(orientation) * self.delta_t, 0],
            [0, self.delta_t]
        ])

        # Covariance prediction
        self.covariance = Fp @ self.covariance @ Fp.T + Fu @ self.Q @ Fu.T


    def update(self):
        # Measurement residual
        H = np.eye(3)
        residual = self.measurement - H @ self.state
        residual[2] = (residual[2] + np.pi) % (2 * np.pi) - np.pi  # Normalize orientation

        # Kalman Gain
        S = H @ self.covariance @ H.T + self.R_lidar
        K = self.covariance @ H.T @ np.linalg.inv(S)

        # Update state and covariance
        self.state = self.state + K @ residual
        self.covariance = (np.eye(3) - K @ H) @ self.covariance

        print("State:", self.state)
        #print("Covariance:", self.covariance)
    
    def main(self):
       
        if self.initialized_imu and self.initialized_lidar:
                self.predict()
                self.update()
    
    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.main()
            rate.sleep()
            self.publish_odometry()
               
        
    

    def publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Fill Pose
        odom.pose.pose.position.x = self.state[0]
        odom.pose.pose.position.y = self.state[1]
        quaternion = quaternion_from_euler(0, 0, self.state[2])
        odom.pose.pose.orientation = Quaternion(*quaternion)

        # Construct the full 6x6 covariance matrix from the 3x3 covariance
        full_covariance = np.zeros((6, 6))
        full_covariance[0:2, 0:2] = self.covariance[0:2, 0:2]  # x, y
        full_covariance[5, 5] = self.covariance[2, 2]  # yaw (orientation)

        # Flatten the 6x6 matrix to a list as expected by ROS
        odom.pose.covariance = list(full_covariance.flatten())

        # Publish
        self.odom_pub.publish(odom)


if __name__ == '__main__':
    try:
        ekf_simplified = EKFSimplified()
        ekf_simplified.spin()
    except rospy.ROSInterruptException:
        pass
