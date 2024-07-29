import rospy
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
import numpy as np

def vid_to_fox():
    video_index=int(input("enter the camera index"))
    vid=cv.VideoCapture(video_index)

    vid_topic=str("video"+str(video_index)+"imgmsg")
    pub = rospy.Publisher(vid_topic,Image,queue_size=10)
    rospy.init_node('vid_to_fox',anonymous=True)
    bridge=CvBridge()
    while (True):
        ret, frame = vid.read()
        frame=np.asanyarray(frame)
        
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

