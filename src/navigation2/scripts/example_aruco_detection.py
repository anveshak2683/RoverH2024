#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
import numpy as np

class Aruco_detection():

    def __init__(self):
        rospy.Subscriber('/zed2i/zed_node/rgb/image_rect_color', Image, self.color_callback)
        self.color_image = np.zeros((360,640))
    def color_callback(self, data):
        try:
            bridge = CvBridge()
            self.color_image = bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)

    def detect_aruco_markers(self):

        # Convert the image to grayscale
        gray = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2GRAY)

        # Load the Aruco dictionary and parameters
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)

        # Detect the markers in the image
        corners, ids, rejected = detector.detectMarkers(gray)

        if ids is not None:
            # Draw the detected markers
            cv2.aruco.drawDetectedMarkers(self.color_image, corners, ids)

            # Display the result
            cv2.imshow("Detected Aruco Markers", self.color_image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                return

        else:
            print("No Aruco markers detected.")
    
    def spin(self):
        while not rospy.is_shutdown():
            self.detect_aruco_markers()
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("aruco_detection")
    rate = rospy.Rate(10)
    run = Aruco_detection()
    run.spin()
