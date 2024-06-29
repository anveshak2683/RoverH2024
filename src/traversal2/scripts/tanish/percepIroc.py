import cv2
import numpy as np

def mask_red(color_image):

    pose = []
    cX, cY = 0, 0
    red_lower = np.array([105, 45, 45])
    red_upper = np.array([140, 255, 255])

    img_hsv = cv2.cvtColor(color_image,cv2.COLOR_BGR2HSV)

    red_mask = cv2.inRange(img_hsv, red_lower, red_upper)

    red_result = cv2.bitwise_and(color_image, color_image, mask = red_mask)

    blurred = cv2.GaussianBlur(red_result, (21, 21),0)
    ret, thresh = cv2.threshold(blurred, 100, 255,cv2.THRESH_BINARY)
    img_gray = cv2.cvtColor(thresh, cv2.COLOR_BGR2GRAY)
    ret, thresh2 = cv2.threshold((img_gray), 20, 255,cv2.THRESH_BINARY)
    
    #kernel = np.ones((13,13))
    #gradient = cv2.morphologyEx(thresh2, cv2.MORPH_OPEN, kernel)
    #edged = cv2.Canny(gradient,80,100)
    contours,_ = cv2.findContours(thresh2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        #if cv2.contourArea(c) < 900:
            #continue
        M= cv2.moments(c)
        if M["m00"]!=0:
            cX = int(M["m10"]/M["m00"])
            cY = int(M["m01"]/M["m00"])
        else:
            cX,cY = 0,0
        pose.append([cX,cY])
        cv2.circle(color_image,(cX,cY),3,(255,0,0),-1)
    cv2.drawContours(color_image, contours, -1, (0, 255, 0), 2)
    cv2.imshow("window", color_image)
    cv2.waitKey(1)
    return cX, cY
