# Anveshak-main.py-documentation

This script is used to detect a red goal cylinder and send the coordinates to the arm for IK.

---

## Class Atrributes:
- `self.is_identified` - bool variable to check if red cylinder detected
- `self.goal_reached` - bool variable to check if close enough to goal linearly
- `self.goal_reached_angular` - bool variable to check if rover is aligned with goal
- `self.p_x,self.p_y` - coordinates of centroid of detected by mask_red function
- `self.cord_x1, self.cord_y1, self.cord_z1` - unused variables
- `self.cv_image` - image from zed2i rgb camera
- `self.image_arrived` - bool variable to check if image from rgb camera arrived
- `self.main_control` - Autonomous task consists of various stages, this shows that the control shifted to cylinder detection 
- `self.depth` - depth of self.p_x,self.p_y from rover(uncorrected)
- `self.cord_x,self.cord_y,self.cord_z` = depth of centroid of object from rover after appyling correction

---

## Variables
- `g` - object of class WheelRPM ROS msg to set velocity 
- `goal_coord` - numpy array of `self.cord_x1, self.cord_y1, self.cord_z1`
- `msg.data[1],msg.data[2],msg.data[0]` - z,y,x coordinates offset taking poisition of zed2i camera into effect
- `d` - 3D distance of centroid from rover using depth coordinates
- `kp` - proportionality constant

---

## Publishers
* `self.tube_pose_pub`: Not used in code.
* `self.vel_pub`: Publishes `msg` array containing goal_coordinates to arm for IK.
* `self.bool_pub`: Provides boolean value if arm should start IK.
* `self.velocity_pub`: Publishes linear and angular velocities to "motion" topic.

---

## Subscribers:
* `self.image_sub`: Subscribes to zed2i node giving colour image data.
* `self.depth_sub`: Subscribes to zed2i nodes giving depth data.

### Publisher and Subscriber Graph
![Screenshot 2024-07-21 at 4 42 40 PM](https://github.com/user-attachments/assets/0be47b78-f647-4929-8bd0-241c61600ad9)

---

## Class Methods:
* `__init__`: initalises the 4 publisher and 2 subscribers mentioned above. It also initialises a nunmber of class attributes listed above.
* `main_callback`: unused function
* `Depth`: callback function for zed2i depth camera subscriber(`self.depth_sub`). It bridges the depth camera data to cv2 thorugh `cv_depth` and then sets `self.depth` to the
  depth of the coordintaes `self.p_x,self.p_y`(centroid detected by `mask_red` function).
* `show_coordinates`: applies a correction formula using the intrinsic paramters of the camera on `self.depth` to get the exact x,y,z coordinates of the centroid of the object from rover
* `callback` callback function for rectified image from zed2i camera. It bridges the data to cv2 through `self.cv_image`.
* `tube_frame`: unused function.
* `mask_red`: function to detect centroid of red object. It converts the image to hsv and detects contous in the upper and lower hsv values of red used. It also Binarizes the image obtained. It then uses **cv.moments** to find the centroid of each object with non zero area and adds them to a list. It however returns only the last value of the centroid found. It also draws a circle and the countours although the image is never displayed.

---

## main() - Control FLow:
- This is the method which controls the flow of the code.
1. It checks if images are identified using `self.is_identified` attribute, and moves with a constant linear velocity if no image is found.
2. If camera detects images, the control goes to `mask_red` method, which obtains 2D centroid values and stores them in `p_x` and `p_y`.
3. Provided valid centroid values are obtained, `show_coordinates` method is called to get the 3D coordinates of the centroid of the cylinder and they are stored in `goal_coord` array.
4. It checks if the control has shifted to cylinder detection mode using `self.main_control`.
5. A `Float32MultiArray()` named `msg` is created with the size and stride equal to the length of the array. The goal coordinates are stored in `msg` array with some offset values to account for the position of camera relative to object.
6. Based on the z coodinate(obtained from msg), `g` gets a certain velocity. The z coordinate points to the depth or the distance between the camera and the cylinder. Based on how far the camera is from the cylinder, `g` is given different velocities. Once the z coordinate is small enough `self.goal_reached` is set to `True`.
7. Similarly, based on how far left or right the centre of the cylinder is(obtained from the x coordinate of the centre of the cylinder) `g` is given a certain angular velocity. When the centre of the cylinder is directly in front of the camera `self.goal_reached_angular` is set to `True`.
8. `g` is published to `self.velocity_pub` to control wheel motion. If the x and z coordinates are non-zero, then `msg` array is published to `self.vel_pub` and `True` is published to `self.bool_pub`.
   
The below flowchart demonstrates the motion of rover based on goal coordinates.
![Flowchart_angular(1)](https://github.com/user-attachments/assets/e8061274-9170-47af-8762-3119019148fb)





