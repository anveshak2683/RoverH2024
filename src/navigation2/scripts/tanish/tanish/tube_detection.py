import rospy
import numpy as np
import cv2
from PIL import Image

#red = [0, 255, 255]  # yell5ow in BGR colour space


#cap = cv2.VideoCapture(0)
#ret, frame = cap.read()


'''def get_limits(self):

    # here insert the bgr values which u want to convert to hsv
    c = np.uint8([[self.color]])
    hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)
    print(hsvC)

    lowerLimit = hsvC[0][0][0] - 0, 200, 150
    upperLimit = hsvC[0][0][0] + 20, 255, 255

    lowerLimit = np.array(lowerLimit, dtype=np.uint8)
    upperLimit = np.array(upperLimit, dtype=np.uint8)

    return lowerLimit, upperLimit'''


# print(get_limits(yellow))
def mask_red(frame):
    hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # lowerLimit, upperLimit = get_limits(color=yellow)
    #for red
    lowerLimit = np.array([0, 200, 150])
    upperLimit = np.array([10, 255, 255])
    #for green
    #lowerLimit=np.array([50,240,240])
    #upperLimit=np.array([70,255,255])
    mask = cv2.inRange(hsvImage, lowerLimit, upperLimit)

    mask_ = Image.fromarray(mask)

    bbox = mask_.getbbox()

    if bbox is not None:
        x1, y1, x2, y2 = bbox
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 0), 5)
        p_x=(x1+x2)/2
        p_y=(y1+y2)/2
        return p_x,p_y
    else:
        return 0,0




    '''cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()'''


# def mask_red(frame):
'''while True:
    #ret, frame = cap.read()
    h = mask_red(frame)
    if h == 1:
        break


cap.release()

cv2.destroyAllWindows()'''

# mask_red(frame)
