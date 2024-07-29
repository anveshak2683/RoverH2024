import cv2 as cv
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np

class ZedCam:
    def __init__(self):
        print("Initializing ZED Camera Node")
        rospy.Subscriber('/zed2i/zed_node/left/image_rect_color', Image, self.image_callback)
        #rospy.Subscriber('/zed2i/zed_node/confidence/confidence_map', Image, self.image_callback)
        self.bridge = CvBridge()  # Initialize CvBridge once
        self.image = None
        self.image_arrived = False

    def image_callback(self, data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, 'bgr8')  # Changed to 'bgr8'
            self.image_arrived = True
        except CvBridgeError as e:
            print(e)

    def main(self):
        if self.image_arrived:
            cv.imshow('ZED Camera', self.image)
            self.image_arrived = False
            if cv.waitKey(1) & 0xFF == ord('q'):
                cv.destroyAllWindows()

    def spin(self):
        rate = rospy.Rate(10)  
        while not rospy.is_shutdown():
            self.main()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node('zed_camera_node', anonymous=True)
    zed_cam = ZedCam()
    print("Spinning ZED Camera Node")
    zed_cam.spin()

