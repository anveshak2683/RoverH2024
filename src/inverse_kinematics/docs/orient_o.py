import cv2 as cv
import numpy as np
import math
import rospy
from math import pi
from std_msgs.msg import Float32MultiArray, Bool, Int32MultiArray, MultiArrayLayout, MultiArrayDimension
import sys


class orientation():
	
	def __init__(self):
		self.vel_pub = rospy.Publisher("auto_arm_signals", Int32MultiArray, queue_size = 10)
		self.goal_reached_pub = rospy.Publisher("om_bool", Bool, queue_size = 10)
		self.enc=rospy.Subscriber("/enc_drive", Float32MultiArray, self.enc_callback)
		self.fin=rospy.Subscriber("ik_over_ah", Bool, self.ik_callback)
		self.angle = None
		self.angle_temp =None
		self.roll_constant =0   # change this according to whatever comes from rostopic of /enc_drive
		self.goal_reached_pub.publish(False)
		self.ret = False
		self.enc_angle = 0.0
		self.ik_bool = False
		self.a = 1
		self.rate = rospy.Rate(10)
  
	def ik_callback(self, msg):	# Callback to start the gripping motion
		self.ik_bool = msg.data

	def enc_callback(self, msg):	#Callback from drive
		self.enc_angle = msg.data[self.roll_constant]
  
  
	def drawAxis(self, img, p_, q_, colour, scale):		#to draw a lines along the desired object
		p = list(p_)
		q = list(q_)
		angle_temp = math.atan2(p[1] - q[1], p[0] - q[0])
		hypotenuse = math.sqrt((p[1] - q[1]) ** 2 + (p[0] - q[0]) ** 2)
		q[0] = p[0] - scale * hypotenuse * math.cos(angle_temp)
		q[1] = p[1] - scale * hypotenuse * math.sin(angle_temp)
		cv.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv.LINE_AA)
		p[0] = q[0] + 9 * math.cos(angle_temp + pi / 4)
		p[1] = q[1] + 9 * math.sin(angle_temp + pi / 4)
		cv.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv.LINE_AA)
		p[0] = q[0] + 9 * math.cos(angle_temp - pi / 4)
		p[1] = q[1] + 9 * math.sin(angle_temp - pi / 4)
		cv.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv.LINE_AA)

	def getOrientation(self, pts, img):		#to get the orientation of a contour
		sz = len(pts)	#finding the number of points in the contour
		data_pts = np.empty((sz, 2), dtype=np.float64)	#initializing a numpy array to store the coordinates of contour points
		for i in range(data_pts.shape[0]):
			data_pts[i, 0] = pts[i, 0, 0]
			data_pts[i, 1] = pts[i, 0, 1]
		mean = np.empty((0))
		mean, eigenvectors, eigenvalues = cv.PCACompute2(data_pts, mean)	#eigenvector correspoding to the highest eigenvalue is the orientation in PCA calculation
		# PCA compute makes covariance matrix x* x(transpose) and gets its eigen values and eigen vecots 
		cntr = (int(mean[0, 0]), int(mean[0, 1]))               # finds center of the object using mean of the data points 
		cv.circle(img, cntr, 3, (255, 0, 255), 2)
		p1 = (cntr[0] + 0.02 * eigenvectors[0, 0] * eigenvalues[0, 0], cntr[1] + 0.02 * eigenvectors[0, 1] * eigenvalues[0, 0])   # uses the largest eigenvalues corresponding eigenvector to get length
		p2 = (cntr[0] - 0.02 * eigenvectors[1, 0] * eigenvalues[1, 0], cntr[1] - 0.02 * eigenvectors[1, 1] * eigenvalues[1, 0])  # uses the second largest eigenvalues corresponding eigenvector to get width	
		self.drawAxis(img, cntr, p1, (0, 255, 0), 1)          # draws the required axes in lenght
		self.drawAxis(img, cntr, p2, (255, 255, 0), 5)        # draws perpendicular axes in width 
		angle = math.atan2(eigenvectors[0, 1], eigenvectors[0, 0])
		return angle
	    
	def roll_controller(self):	         # controls how much to change roll on the arm
		if not math.isnan(self.angle):   # nan means "not a number" , hence this if statement executes when angle is a number 
			msg=Int32MultiArray() 
			msg.data=[0,0,0,0,0,0]              # creates a new multi array and initialises data field with zeroes
			msg.layout = MultiArrayLayout()
			msg.layout.data_offset = 0
			msg.layout.dim = [MultiArrayDimension()]
			msg.layout.dim[0].size = msg.layout.dim[0].stride = len(msg.data)
			msg.layout.dim[0].label = 'write'

			print("Angle:", self.angle)
			if self.enc_angle < self.angle - 2:   # defining this as anticlockwise rotation
				msg.data[5] = -255
				print("roll angle:", self.enc_angle)        # print the current roll angle received from encoder 
			elif self.enc_angle > self.angle + 2:                # if difference in current angle and instructed angle has a difference greater than 2
				msg.data[5] = 255
				print("roll angle:", self.enc_angle)          # print the current roll angle received from encoder 
				self.rate.sleep()
			self.vel_pub.publish(msg)
			gripper_pub = 3 #set this to actual gripper publishing value while testing
			if abs(self.enc_angle - self.angle) < 2:
				msg.data[gripper_pub] = 255  #determine sign as per closing and opening
				self.vel_pub.publish(msg)
				rospy.sleep(5)
				self.goal_reached_pub.publish(True)
	           
	    	           

	def main(self):
		print("inside main")
		cap = cv.VideoCapture(3)	#capturing video from the camera
		if not cap.isOpened():
			print("Error: Failed to open camera.")
			exit()
		while True:
			ret, frame = cap.read()
			if not ret:
				print("Error: Failed to capture frame.")
				break
					
	############# This Part is for detection and can be edited according to the object you want ###################################

			hsvFrame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
			lower = np.array([0, 70, 50], np.uint8)  #ask pranav which object is this used to detect
			upper = np.array([10, 255, 255], np.uint8)

            #This is for orange colour cone
            #lower = np.array([0, 55, 228], np.uint8)
            # upper = np.array([21, 255, 255], np.uint8)
            
			mask = cv.inRange(hsvFrame, lower, upper)
			red_output = cv.bitwise_and(frame, frame, mask=mask)
			gray = cv.cvtColor(red_output, cv.COLOR_BGR2GRAY)
			_, thresh = cv.threshold(gray, 10, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)
			contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)	#finding contours based on the threshold
			angle_memory = None
			if self.angle != None:
				angle_memory = self.angle
			for i, c in enumerate(contours):	# enumerate() gives us the index and value of an element in an array
				area = cv.contourArea(c)           
				if area < 1e4:
					continue
				else:
					print(area)
					cv.drawContours(frame, contours, i, (0, 255, 255), 4)
					angle = self.getOrientation(c, frame)
					
					if self.a != 0 and angle != 0.0:
						rospy.sleep(2)
						self.angle = angle * 180/pi	#getting the angle in degrees
						self.a = 0
					if self.angle > 0:	#offsetting
						self.angle = 90 - self.angle
					elif self.angle < 0:
						self.angle = -90 - self.angle
					
					self.ret = True
			if angle_memory != None and self.angle != None:	#NAggi maalum kya hotha hai
				if angle_memory + self.angle <= 91 and angle_memory + self.angle >= 89:
					if angle_memory < self.angle:
						self.angle = angle_memory
				elif angle_memory + self.angle >=-91 and angle_memory + self.angle <= -89:
					if angle_memory < self.angle:
						self.angle = angle_memory
				else:
					self.angle = angle_memory
						
			print("Angle:", self.angle)
			if self.ret == True and self.ik_bool == True:
				self.roll_controller()
			small_res = cv.resize(frame, (640, 480))	#resizing the image and printing it
			cv.imshow('contours', small_res)
			if cv.waitKey(1) & 0xFF == ord('q'):
				break
		cap.release()	#closing video feed
		cv.destroyAllWindows()
	def spin(self):
		while not rospy.is_shutdown():
			self.main()
			rate.sleep()
			
if __name__ == '__main__':
	try:
		rospy.init_node("bruh", anonymous = True)
		rate = rospy.Rate(10)
		run = orientation()
		run.spin()
	except KeyboardInterrupt:
		sys.exit()



