import cv2 as cv
import numpy as np
import math
import rospy
from math import pi
from std_msgs.msg import Float32MultiArray, Bool, Int32MultiArray, MultiArrayLayout, MultiArrayDimension
import sys


class orientation():
	
	def __init__(self):
		print("hi")
		self.vel_pub = rospy.Publisher("auto_arm_signals", Int32MultiArray, queue_size = 10)
		self.om_pub = rospy.Publisher("om_bool", Bool, queue_size = 10)
		rospy.Subscriber("/enc_drive", Float32MultiArray, self.enc_callback)
		rospy.Subscriber("ik_over_ah", Bool, self.pranav_callback)
		self.angle = None
		self.roll_constant =0   # change this according to whatever comes from rostopic of /enc_drive
		self.om_pub.publish(False)
		self.ret = False
		self.enc_angle = 0.0
		self.pranav_bool = False
		self.a = 1
		self.rate = rospy.Rate(10)
	def pranav_callback(self, msg):
		self.pranav_bool = msg.data

	def enc_callback(self, msg):
		self.enc_angle = msg.data[self.roll_constant]
	def drawAxis(self, img, p_, q_, colour, scale):
		p = list(p_)
		q = list(q_)
		angle = math.atan2(p[1] - q[1], p[0] - q[0])
		hypotenuse = math.sqrt((p[1] - q[1]) ** 2 + (p[0] - q[0]) ** 2)
		q[0] = p[0] - scale * hypotenuse * math.cos(angle)
		q[1] = p[1] - scale * hypotenuse * math.sin(angle)
		cv.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv.LINE_AA)
		p[0] = q[0] + 9 * math.cos(angle + pi / 4)
		p[1] = q[1] + 9 * math.sin(angle + pi / 4)
		cv.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv.LINE_AA)
		p[0] = q[0] + 9 * math.cos(angle - pi / 4)
		p[1] = q[1] + 9 * math.sin(angle - pi / 4)
		cv.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv.LINE_AA)

	def getOrientation(self, pts, img):
		sz = len(pts)
		data_pts = np.empty((sz, 2), dtype=np.float64)
		for i in range(data_pts.shape[0]):
			data_pts[i, 0] = pts[i, 0, 0]
			data_pts[i, 1] = pts[i, 0, 1]
		mean = np.empty((0))
		mean, eigenvectors, eigenvalues = cv.PCACompute2(data_pts, mean)
		cntr = (int(mean[0, 0]), int(mean[0, 1]))
		cv.circle(img, cntr, 3, (255, 0, 255), 2)
		p1 = (cntr[0] + 0.02 * eigenvectors[0, 0] * eigenvalues[0, 0], cntr[1] + 0.02 * eigenvectors[0, 1] * eigenvalues[0, 0])
		p2 = (cntr[0] - 0.02 * eigenvectors[1, 0] * eigenvalues[1, 0], cntr[1] - 0.02 * eigenvectors[1, 1] * eigenvalues[1, 0])
		self.drawAxis(img, cntr, p1, (0, 255, 0), 1)
		self.drawAxis(img, cntr, p2, (255, 255, 0), 5)
		angle = math.atan2(eigenvectors[0, 1], eigenvectors[0, 0])
		return angle
	    
	def roll_controller(self):
		if not math.isnan(self.angle):
			msg=Int32MultiArray()
			msg.data=[0,0,0,0,0,0]
			msg.layout = MultiArrayLayout()
			msg.layout.data_offset = 0
			msg.layout.dim = [MultiArrayDimension()]
			msg.layout.dim[0].size = msg.layout.dim[0].stride = len(msg.data)
			msg.layout.dim[0].label = 'write'

			print("Angle:", self.angle)
			if self.enc_angle < self.angle - 2:   # defining this as anticlockwise rotation
				msg.data[5] = -255
				print("roll angle:", self.enc_angle)
			elif self.enc_angle > self.angle + 2:
				msg.data[5] = 255
				print("roll angle:", self.enc_angle)
				self.rate.sleep()
			self.vel_pub.publish(msg)
			gripper_pub = 3 #set this to actual gripper publishing value while testing
			if abs(self.enc_angle - self.angle) < 2:
				msg.data[gripper_pub] = 255  #determine sign as per closing and opening
				self.vel_pub.publish(msg)
				rospy.sleep(5)
				self.om_pub.publish(True)
	           
	    	           

	def main(self):
		print("inside main")
		cap = cv.VideoCapture(3)
		if not cap.isOpened():
			print("Error: Failed to open camera.")
			exit()
		while True:
			ret, frame = cap.read()
			if not ret:
				print("Error: Failed to capture frame.")
				break
						##############################################################################
	############# This Part is for detection and can be edited according to the object you want ###################################

			hsvFrame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
			lower = np.array([0, 70, 50], np.uint8)
			upper = np.array([10, 255, 255], np.uint8)

                                                               #This is for orange colour cone
                #lower = np.array([0, 55, 228], np.uint8)
                # upper = np.array([21, 255, 255], np.uint8)
			mask = cv.inRange(hsvFrame, lower, upper)
			red_output = cv.bitwise_and(frame, frame, mask=mask)
			gray = cv.cvtColor(red_output, cv.COLOR_BGR2GRAY)
			_, thresh = cv.threshold(gray, 10, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)
			contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
			angle_memory = None
			if self.angle != None:
				angle_memory = self.angle
			for i, c in enumerate(contours):
				area = cv.contourArea(c)           
				if area < 1e4:
					continue
				if area > 1e4:
					print(area)
					cv.drawContours(frame, contours, i, (0, 255, 255), 4)
					
                    		################################################################################
					angle = self.getOrientation(c, frame)
					
					if self.a != 0 and angle != 0.0:
						rospy.sleep(2)
						self.angle = angle * 180/pi
						self.a = 0
					if self.angle > 0:
						self.angle = 90 - self.angle
					elif self.angle < 0:
						self.angle = -90 - self.angle
					
					self.ret = True
			if angle_memory != None and self.angle != None:
				if angle_memory + self.angle <= 91 and angle_memory + self.angle >= 89:
					if angle_memory < self.angle:
						self.angle = angle_memory
				elif angle_memory + self.angle >=-91 and angle_memory + self.angle <= -89:
					if angle_memory < self.angle:
						self.angle = angle_memory
				else:
					self.angle = angle_memory
						
			print("Angle:", self.angle)
			if self.ret == True and self.pranav_bool == True:
				self.roll_controller()
			small_res = cv.resize(frame, (640, 480))
			cv.imshow('contours', small_res)
			if cv.waitKey(1) & 0xFF == ord('q'):
				break
		cap.release()
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



