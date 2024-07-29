import cv2
import numpy as np
import os

def color_name(degrees):
    if 0 <= degrees < 15 or 345 <= degrees <= 360:
        return "Red"
    elif 15 <= degrees < 45:
        return "Orange"
    elif 45 <= degrees < 75:
        return "Yellow"
    elif 75 <= degrees < 150:
        return "Green"
    elif 150 <= degrees < 225:
        return "Blue"
    elif 225 <= degrees < 285:
        return "Purple"
    elif 285 <= degrees < 345:
        return "Pink"

image_paths = ['~/bluebox.png']

for img_path in image_paths:
    img_path = os.path.expanduser(img_path)
    
    img = cv2.imread(img_path)
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    blur = cv2.GaussianBlur(hsv_img, (9, 9), 1)
    
    line_index = 50
    line_pixels = hsv_img[line_index, :]
    
    nonredpixel = np.array([])
    pixels1 = np.array([])
    pixels2 = np.array([])

    for i in line_pixels:
        if i[1] >= 50: 
            if i[0] >= 7.5  and i[0] <= 172.5: 
                nonredpixel = np.append(nonredpixel, i) 

    nonredpixel = nonredpixel.reshape((-1, 3))
    if nonredpixel.size == 0:
        nonredpixel = np.append(nonredpixel,[0,0,0])
    for i in line_pixels:
        if 0 <= i[0] <= 7.5 and i[1]>=50:
            pixels1 = np.append(pixels1,i)
        if 172.5 <= i[0] <= 180 and i[1]>=50:
            pixels2 = np.append(pixels2,i)  
    pixels1 = pixels1.reshape((-1, 3))
    if pixels1.size == 0:
        pixels1 = np.append(pixels1,[0,0,0])

    pixels2 = pixels2.reshape((-1, 3))
    if pixels2.size == 0:
        pixels2 = np.append(pixels2,[0,0,0])

    print(nonredpixel)
    print(pixels1)
    print(pixels2)

    avg_color = np.mean(nonredpixel, axis=0)
    if not np.all(avg_color == 0.0):
       avg_color = np.mean(nonredpixel, axis=0)
    else:
        avg_color = np.array([0,0,0])

    avg_red1 = np.mean(pixels1, axis=0)
    if not np.all(avg_red1 == 0.0):
       avg_red1 = np.mean(pixels1, axis=0)
    else:
        avg_red1 = np.array([0,0,0])

    avg_red2 = np.mean(pixels2, axis=0)
    if not np.all(avg_red2 == 0.0):
       avg_red2 = np.mean(pixels2, axis=0)
    else:
        avg_red2 = np.array([181,0,0])

    print(avg_color)
    print(avg_red1)
    print(avg_red2)
    avg_hue = avg_color[0] * 2 *len(nonredpixel)
    avg_red1_hue = avg_red1[0] * 2*len(pixels1) 
    avg_red2_hue = (360 -(avg_red2[0] * 2))*len(pixels2)

    avg_color_hue=(avg_red1_hue +avg_red2_hue+avg_hue)/(len(nonredpixel)+len(pixels1)+len(pixels2))
    print(avg_hue)
    print(avg_red1_hue)
    print(avg_red2_hue)

    color = color_name(avg_color_hue)

    #print(f"Average hsv of line {line_index}: {avg_color}")
    print(f"Average hue in degrees: {avg_color_hue}, AVG_COLOUR: {color}")
    
    cv2.line(hsv_img, (0, line_index), (260, line_index), (0, 255, 0), 2)
    cv2.imshow("Image", hsv_img)
    cv2.waitKey(0)

cv2.destroyAllWindows()
