import cv2 as cv
import numpy as np
import math
import rospy
from std_msgs.msg import Int32MultiArray, MultiArrayLayout, MultiArrayDimension, Bool, Int32
from navigation2.msg import red
pi = math.pi

class orient():
    def __init__(self):
        self.ik_pause = False
        self.orientation_done = False
        self.pause_pub = rospy.Publisher('/ik_pause', Bool, queue_size=10)
        rospy.Subscriber('/ik_pause', Bool, self.pause_callback)
        rospy.Subscriber('/tanish_red', red, self.tanish_callback)
        self.stop_pub = False
        self.counter = 0
        self.control = False
        #self.final_angle = 0
        #self.final_stop = 0

    def tanish_callback(self, msg):
        self.control = msg.detect

    def pause_callback(self, data):
        self.ik_pause = True

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

if __name__ == '__main__':
    rospy.init_node("omorient")
    #pub = rospy.Publisher("/auto_arm_signals", Int32MultiArray, queue_size=10)
    #final_angle = 100
    #final_stop = 0
    orient_pub = rospy.Publisher('/orientation', Int32, queue_size = 10)
    orient_obj = orient()
    cap = cv.VideoCapture(2)
    roll = 0

    if not cap.isOpened():
        print("Error: Failed to open camera.")
        exit()

    while not rospy.is_shutdown():
        ret, frame = cap.read()

        if not ret:
            print("Error: Failed to capture frame.")
            break

        hsvFrame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        lower = np.array([0, 50, 100], np.uint8)
        upper = np.array([10, 255, 255], np.uint8)
        mask = cv.inRange(hsvFrame, lower, upper)
        red_output = cv.bitwise_and(frame, frame, mask=mask)
        gray = cv.cvtColor(red_output, cv.COLOR_BGR2GRAY)
        _, thresh = cv.threshold(gray, 10, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)
        contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        angle = 0

        for i, c in enumerate(contours):
            area = cv.contourArea(c)           
            if area < 1e4:
                continue
            if area > 1e4:
                print(area)
                cv.drawContours(frame, contours, i, (0, 255, 255), 4)
                angle = orient_obj.getOrientation(c, frame)
                angle = (1.57 - angle) * 180 / pi
                if abs(angle) > 90:
                    angle = 0
        
        print("Angle:", angle)
        #if abs(angle) >10 and abs(angle) < 90:
            #final_angle = angle
            #final_stop = final_stop +1
        msg = Int32MultiArray()
        msg.layout = MultiArrayLayout()
        msg.layout.data_offset = 0
        msg.layout.dim = [MultiArrayDimension()]
        msg.layout.dim[0].size = msg.layout.dim[0].stride = 6
        msg.layout.dim[0].label = 'write'
        #print("final stop angle", final_stop, final_angle)
        if angle >= 30 and orient_obj.counter == 0:
            #msg.data = [0, 0, 255, 0, 0, 0]
            roll = 255
            orient_obj.orientation_done = False
            orient_obj.stop_pub = False
        elif angle <= -30 and orient_obj.counter == 0:
            #msg.data = [0, 0, -255, 0, 0, 0]
            roll = -255
            orient_obj.orientation_done = False
            orient_obj.stop_pub = False
        elif orient_obj.counter == 0 and abs(angle) <30:
#            msg.data = [0, 0, 0, 0, 0, 0]
            roll = 0
            #pub.publish(msg)
            orient_obj.orientation_done = True
            orient_obj.ik_pause = False
            #orient_obj.pause_pub.publish(False)
            orient_obj.stop_pub = True
            orient_obj.counter = 1
        #print(orient_obj.ik_pause, orient_obj.orientation_done, orient_obj.stop_pub)    
        if orient_obj.control == True:
            orient_pub.publish(roll)
        else:
            roll = 0
            orient_pub.publish(roll)
        print("Published:", msg)
        
        small_res = cv.resize(frame, (640, 480))
        #cv.imshow('contours', small_res)
        #if cv.waitKey(1) & 0xFF == ord('q'):
            #break

    #cap.release()
    #cv.destroyAllWindows()

