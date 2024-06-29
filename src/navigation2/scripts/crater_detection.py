import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator
import time
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

#code to detect curvature of a crater

def find_height2(depth_frame, top, bottom, left, right):
    f_length = 2.1
    pixel_height = (top-bottom) * 0.26
    depth = depth_frame.get_distance(int((left+right)/2),int((top+bottom)/2))
    print("pixel height = ", pixel_height)
    print("depth = ", depth )
    height = pixel_height * depth / f_length
    return height



def detect_crater_curvature(top, bottom, left, right, depth_frame):
    depth_list = []
    # list1 = np.linspace(bottom, top, 1)
    list1 = []
    print("check1")
    print("bottom, top = ", bottom, top)
    for i in range(top, bottom, 1):
        # print("check2")
        list1.append(i)
        depth = depth_frame.get_distance(int((left+right)/2), i)
        depth_list.append(depth)
        #so now you'll have a set of points along with coords
    # print("depth_list = ", depth_list)
    # print("list1 = ", list1)
    coefficients = np.polyfit(list1,depth_list,3)
    polynomial_fitted = np.poly1d(coefficients)
    slope_function = polynomial_fitted.deriv()
    slope_list = slope_function(list1)
    # print(slope_list)
    angle_list = np.arctan2(slope_list, list1)
    print(angle_list)
    for i in slope_list:
        if i>45:
            print("Too steep for us to traverse")

    return angle_list



#model = YOLO('/home/kavin/Downloads/crater_weights/best.pt')  # Replace with the path to your YOLO model
model = YOLO('/home/kavin/Downloads/crater_and_cube_weights/weights/best.pt')

cap = cv2.VideoCapture(0)
# Configure RealSense pipeline to get color frames
# pipeline = rs.pipeline()
# config = rs.config()
#config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
#config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Enable depth stream
#pipeline.start(config)
time.sleep(2)
if not cap.isOpened():
    print("Tryin to open camera . . .") 
    exit()
print("check1")

while True:
    # Wait for coherent frames
    # frames = pipeline.wait_for_frames()
    # color_frame = frames.get_color_frame()
    # depth_frame = frames.get_depth_frame()
    print("hi")
    ret, img = cap.read()

    if not ret:
        print("Error: failed to capture frame")
        break
    # Convert RealSense color frame to OpenCV format
    # img = np.asanyarray(c.get_data())
    #img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Run YOLO model inference
    
    results = model.predict(img, conf=0.55, max_det=2)

    for r in results:
        annotator = Annotator(img)
        boxes = r.boxes
        for box in boxes:
            b = box.xyxy[0]  # get box coordinates in (left, top, right, bottom) format
            c = box.cls
            annotator.box_label(b, model.names[int(c)])

            # Draw rectangle around the object
            left, top, right, bottom = map(int, b)
            cv2.rectangle(img, (left, top), (right, bottom), (0, 255, 0), 2)
            
           # try:
                # height = find_height2(depth_frame, top, bottom, left, right)
               # print("Height of the object:", height)
                # curvature_list = detect_crater_curvature(top, bottom, left, right, depth_frame)
          #  except Exception as e:
          #      print("Error:", e)

    # Show annotated image
    cv2.imshow('YOLO V8 Detection', img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Stop streaming
#pipeline.stop()
cv2.destroyAllWindows()

