import rospy
import sys
import numpy as np
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class Zed2iFilterNode:
    def __init__(self):
        self.out_img = None
        rospy.init_node("zed2i_filter")
        self.bridge = CvBridge()
        self.subscriber = rospy.Subscriber("zed2i/zed_node/depth/depth_registered", Image, self.sub_callback)
        self.publisher = rospy.Publisher("zed2i/zed_node/depth/depth_filtered", Image, queue_size =10)

    def sub_callback(self, img):
        try:
            print("say1")
            self.out_img = self.bridge.imgmsg_to_cv2(img, "passthrough")
            print(self.out_img)
            print("say2")
        except Exception as e:
            print(e)
        
    def timer_callback(self):
        if self.out_img is not None:
            img_data = self.out_img
            self.out_img = np.zeros_like(img_data)
            for i in range(360):
                for j in range(640):
                    print(i,j)
                    if not (np.isnan(img_data[i][j]) and np.isinf(img_data[i][j])):
                        self.out_img.data[i][j] = img_data[i][j]
            img_msg = self.bridge.cv2_to_imgmsg(self.out_img, "passthrough")
            self.publisher.publish(img_msg)
        
    def spin(self):
        while not rospy.is_shutdown():
            self.timer_callback()

if __name__ == "__main__":
    try:
        node = Zed2iFilterNode()
        node.spin()
    except KeyboardInterrupt:
        sys.exit()	
