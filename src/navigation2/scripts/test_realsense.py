import time
import numpy as np
import cv2
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


#cap=cv2.VideoCapture(2)

class track():
    def __init__(self):
        rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback)
        self.color_came= False
        self.depth_came = False
        self.depth_color = 0.0
        self.color_detected = False
        self.color_came= False

    def color_callback(self, data):
            try:
                bridge = CvBridge()
                self.color_image = bridge.imgmsg_to_cv2(data, 'bgr8')
                self.color_came = True
            except CvBridgeError as e:
                    print(e)

    def nothing(x):
            pass
    
    cv2.namedWindow("Trackbars")
    cv2.createTrackbar("L - H", "Trackbars", 0, 179, nothing)
    cv2.createTrackbar("L - S", "Trackbars", 0, 255, nothing)
    cv2.createTrackbar("L - V", "Trackbars", 0, 255, nothing)
    cv2.createTrackbar("U - H", "Trackbars", 179, 179, nothing)
    cv2.createTrackbar("U - S", "Trackbars", 255, 255, nothing)
    cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)


    def color_detection(self): 
            while True:
                frame=self.color_image
                frame=cv2.resize(frame,(640,480))
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                
                l_h = cv2.getTrackbarPos("L - H", "Trackbars")
                l_s = cv2.getTrackbarPos("L - S", "Trackbars")
                l_v = cv2.getTrackbarPos("L - V", "Trackbars")
                u_h = cv2.getTrackbarPos("U - H", "Trackbars")
                u_s = cv2.getTrackbarPos("U - S", "Trackbars")
                u_v = cv2.getTrackbarPos("U - V", "Trackbars")
                lower_blue = np.array([l_h, l_s, l_v])
                upper_blue = np.array([u_h, u_s, u_v])
                mask = cv2.inRange(hsv, lower_blue, upper_blue)
                result = cv2.bitwise_and(frame, frame, mask=mask)    

                # show thresholded image
                cv2.imshow("mask", mask)
                cv2.imshow("result", result)  

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
            #cap.release()
            cv2.destroyAllWindows()

    def main(self):
         if self.color_came==True:
            self.color_detection()
    
    def spin(self):
        while not rospy.is_shutdown():
            self.main()
            rate.sleep()

        else:
            print("hi")


if __name__=="__main__":
    rospy.init_node("Track_Colors", anonymous=True)
    rate = rospy.Rate(10)
    ahh = track()
    ahh.spin()

    
