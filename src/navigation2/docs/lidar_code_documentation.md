# Lidar-Code-Documentation

# Lidar_code_2.py

---

This is a python script which enables the rover to avoid obstacles and process the color and depth of any obstacles in its environment using a zed2i camera and a lidar

---

## Variables and Objects

`lower_range, upper_range` - hsv ranges for 

`self.color_image` - rectified image from zed2i camera

`self.depth_image` - depth image from zed2i camera

`left_num` - starting point for left range of lidar data

`right_num` - starting point for right range of lidar data

`left_check_sum` - sum of lidar data from left_num to left_num+100

`right_check_sum` - sum of lidar data from right_num to right_num+100

`self.left_check` = left_check_sum/100

`self.right_check` = right_check_sum/100

`self.odom_self[0]` - current x position of rover

`self.odom_self[1]` - current y position of rover

`self.initial_odom[0]` - initial x position of rover

`self.initial_odom[1]` - initial y position of rover

`self.color_detected` - true or false based on wether color is detected

`twist` - object of wheel RPM

`safety_distance` - threshold distance from any obstacles

`self.counter` - a counting variable which keeps increasing until 50 (wherein distance travelled is computed and self.counter is reset)

`time.time()` - a global clock

`self.timer` - a temp variable used to store time at certain intervals

`self.color_came` - a bool variable to check if color image has been captured by camera

`self.depth_came` - a bool variable to check if depth info has been captured by depth camera

---

## Publishers and Subscribers:

`self.pub` - rospy publisher which publishes ‘/motion’ topic of type WheelRPM

`self.cam_color` - rospy subscriber subscribing to '/zed2i/zed_node/rgb/image_rect_color’ topic. It is used to colour info from the image captured

`self.cam_depth` - rospy subscriber subscribing to '/zed2i/zed_node/depth/depth_registered’ topic. It is used to get depth info from the image captured

`self.odom` - rospy subscriber subscribing '/odometry/filtered’ topic. It is used to get the rover’s position and configuration in the field

`self.laser` - rospy subscriber subscribing to ‘/scan’ topic. It is used to get data from the lidar.

---

## Class methods:

`__init__` - Initialises Subscribers to /scan(Lidar data), /odometry/filtered(Odometry), /zed2i/zed_node/rgb/image_rect_color(zed2i rectified image) and /zed2i/zed_node/depth/depth_registered(zed2i depth image)

init also initialises a number of class attributes to various values

`color_callback` - Callback function for zed2i rectified image subscriber. It bridges the image from the camera to CV2 through `self.colour_image`

`depth_callback` - Callback function for zed2i depth camera subscriber. It bridges the image from camera to CV2 through `self.depth_color`

`lidar_callback` - Callback function for lidar subscriber. It sets `left_num` and `right_num`, and then iterates through a hundred points for both of them and finds stores the sums as `left_check_sum` and `right_check_sum`. It sets   `self.right_check` and `self.left_check` to their respective sums/100.

`odom_callback` - Callback function for odometer subscriber. It sets the current x and y positions of the rover. If `self.odom_initialized` is false, it sets the current x,y position of the rover. It sets `self.odom_self[0]` and `self.odom_self[1]` to current position - final position

`spline_distance` - Fits a cubic spline to the points passed to it. Returns the length of the arc.

`color_detection` - This function detects all red, blue and yellow which are above a threshold area of 600 units. It then sets the distance of that object in `self.depth_colour`

`main` - The final computation is done in this method and it is comprised of various parts:**

1. The first part of the main method moves the rover ahead while making sure it does not hit any objects by using the lidar code to check which side to turn. If any of the objects is closer than the threshold distance then turn the rover with an angular velocity of 20. It also keeps track of the coordinates of the rover through the lists `self.x` and `self.y`
2. The second part of main calculates the displacement of the rover every two seconds.
3. The third part of main checks if the cameras have received information, and then calls `self.color_detection`. It then checks if `self.counter` has reached 50, if true gives the distance along the cubic spline the rover has traveled till now, followed by resetting `self.counter`
4. The fourth part of main checks if any coloured object is detected and outputs the distance of the object from the rover. It then sets `self.distance` to the sum of displacement of rover and distance of coloured object from rover to ouptput the final distance of coloured object from starting point of rover.

---

##  Flow Control:
![image](https://github.com/user-attachments/assets/1af67a69-148e-4616-87d0-5db57e6f3fa9)

