import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import sys

class imei():
    def __init__(self):
        rospy.Subscriber("/zed2i/zed_node/left/image_rect_color", Image, self.image_callback)
        self.cv_image = None

    def image_callback(self, data):
        try:
            bridge = CvBridge()
            self.cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
#            self.cv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        except Exception as e:
            print(e)

# Convert image to grayscale
# gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRY)

    def cube_detect_cv(self):
        if self.cv_image is not None:
        # Convert image to grayscale
            gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur to reduce noise
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Compute gradients using Sobel operator
            sobel_x = cv2.Sobel(blurred, cv2.CV_64F, 1, 0, ksize=3)
            sobel_y = cv2.Sobel(blurred, cv2.CV_64F, 0, 1, ksize=3)

        # Compute magnitude of gradients
            magnitude = cv2.magnitude(sobel_x, sobel_y)

        # Normalize magnitude to 8-bit range
            magnitude = np.uint8(255 * magnitude / np.max(magnitude))

        # Perform thresholding to obtain a binary image
            _, thresholded = cv2.threshold(magnitude, 50, 255, cv2.THRESH_BINARY)

        # Find contours in the thresholded image
            contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Iterate through detected contours
            for contour in contours:
            # Approximate the contour to a simpler polygon
                approx = cv2.approxPolyDP(contour, 0.03 * cv2.arcLength(contour, True), True)
            
            # If the contour has 4 vertices (quadrilateral)
                if len(approx) == 4:
                # Compute the bounding box for the contour
                    (x, y, w, h) = cv2.boundingRect(approx)
                
                # Check if the bounding box is approximately square or rectangular
                    aspect_ratio = float(w) / h
                    if 0.9 <= aspect_ratio <= 1.1:
                    # Draw the bounding box around the contour
                        cv2.rectangle(self.cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.putText(self.cv_image, 'Cube', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Display the output image
            cv2.imshow('Cube Detection', self.cv_image)
            cv2.waitKey(1)  # Adjust the waitKey value to control the display time


    def spin(self):
        while not rospy.is_shutdown():
            self.cube_detect_cv()
            rate.sleep()


if __name__=="__main__":
    try:
        rospy.init_node("ahhhhh")
        rate = rospy.Rate(1)
        run = imei()
        run.spin()
    except KeyboardInterrupt:
        sys.exit()
