from tube_detection import mask_red
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image
#from PIL import Image

from std_msgs.msg import String
from geometry_msgs.msg import Point 
import cv2
import numpy as np
import rospy
import sys
import tf 


from tf.transformations import quaternion_from_euler
from tf import LookupException,ConnectivityException,ExtrapolationException

#run the below five lines to check if masked_red is working or not
#img=cv2.imread("scripts/red_cylinder.jpg")
#mask_red(img)
#cv2.imshow('frame',img)
#cv2.waitKey(0)
#cv2.destroyAllWindows()

class tube_detector:
    def __init__(self):
        self.tube_pose_pub = rospy.Publisher("tube_pose",Point,queue_size=10)
        self.bridge= CvBridge()
        #self.depth_sub=rospy.Subscriber("/galileo/zed2/depth/depth_registered",Image,self.Depth)
        self.image_sub = rospy.Subscriber("/zed2i/zed_node/left/image_rect_color",Image,self.callback)
        self.depth_sub=rospy.Subscriber("/zed2i/zed_node/depth/depth_registered",Image,self.Depth)
        self.p_x, self.p_y = 0,0
        self.cv_image = None

    def Depth(self,data):
        try:
            cv_depth=self.bridge.imgmsg_to_cv2(data,'passthrough')

        except CvBridgeError as e:
            print(e)
        #print(np.ndim(cv_depth))
        #print(cv_depth)
        self.depth=cv_depth[int(self.p_y)][int(self.p_x)]

    def show_coordinates(self,p_x,p_y,cv_image):
        #K matrix values
        f_x=527.2972398956961
        f_y=527.2972398956961
        c_x=658.8206787109375
        c_y=372.25787353515625
    

        self.cord_x=self.depth*(p_x-c_x)/f_x
        self.cord_y=self.depth*(p_y-c_y)/f_y
        self.cord_z=self.depth

        self.font=cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale=0.5
        self.color=[255,0,0]
        self.thickness=1
    

        
    def callback(self,data):
        try:
            cv_image=self.bridge.imgmsg_to_cv2(data,'bgr8')
        except CvBridgeError as e :
            print(e)

        self.p_x,self.p_y=mask_red(cv_image)
        #print(self.p_x,self.p_y)
        if self.p_x is not None:


            self.show_coordinates(self.p_x,self.p_y,cv_image)
            self.tube_frame(self.cord_x,self.cord_y,self.cord_z)
            cv2.putText(cv_image,str((self.cord_x,self.cord_y,self.cord_z)),(int(self.p_x),int(self.p_y)),self.font,self.font_scale,self.color,self.thickness,cv2.LINE_AA)
        else:
            pass
        cv2.imshow('frame',cv_image)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()

    def tube_frame(self,x,y,z):
        
        
        # self.tube_pose_pub.publish(t)
        # print(x, y, z)
        br = tf.TransformBroadcaster()
        br.sendTransform((x,y,z), (0,0,0,1), rospy.Time.now(), "sample_tube", "zed2_left_camera_optical_frame")
        listener = tf.TransformListener()
        rospy.sleep(0.25)
        print(2)
        try:
            (trans,rot)=listener.lookupTransform("base_link","sample_tube",rospy.Time(0))
            print(trans)
            br.sendTransform(trans,(0,0,0,1),rospy.Time.now(),"sample_tube_base","base_link")
        except (LookupException, ConnectivityException, ExtrapolationException):
            print(1)
            pass

        






def main(args):
    td=tube_detector()
    rospy.init_node('tube_detector',anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")

if __name__ == '__main__':
    main(sys.argv)






