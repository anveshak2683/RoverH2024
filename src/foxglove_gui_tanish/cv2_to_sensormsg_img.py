import rospy
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError

def vid_to_fox():
    vid=cv.VideoCapture(1)

    pub = rospy.Publisher("video0_imgmsg",Image,queue_size=10)
    rospy.init_node('vid_to_fox',anonymous=True)
    bridge=CvBridge()
    while (True):
        ret, frame = vid.read()
        
        try:
            ros_image=bridge.cv2_to_imgmsg(frame,'bgr8')
            pub.publish(ros_image)
        except CvBridgeError as e:
            print(e)
            




if __name__=='__main__':
    try:
        vid_to_fox()
    except rospy.ROSInterruptException:
        pass

