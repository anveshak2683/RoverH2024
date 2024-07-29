# FullPotential
## Drive Node for Autonomous Rover
This is a Python script that implements a drive node for an autonomous rover using ROS (Robot Operating System). The node receives joystick input, encoder data, and motion commands, and sends PWM (Pulse Width Modulation) signals to the rover's motors.
## Initialization
### Class Drive
* Initializes the ROS node drive_arc.
* The node is **subscribed** to 'joy' and 'encoder' topics. publishing to 'motor_rpm' topic.
#### Variable used:
 * __self.modeupbtn__ and __self.modednbtn__ : They are used to increment and decrement the pwm values respectively.
 * __self.rotinplace_btn__ : This is a variable(button) responsible for rotating the rover in one place.I have mentioned in breif about this below in the function it is used.
 * __self.steer_unlock_axis__ : This variable is used to unlock the steering control. When the joystick axis with index 4 is manipulated, it toggles the lock state of the steering. This means that the user can enable or disable manual control of the steering.
 * __self.steer_samedir_axis__ : This variable is used to control the steering direction in the same direction for all wheels.This allows the rover to steer left or right uniformly.
 * __self.steer_oppdir_axis__ : This variable is used to control the steering direction in opposite directions for front and rear wheels. This enables the rover to perform more complex maneuvers like rotating in place or crab steering.
 * __self.full_potential_unlock_axis__ : This variable is used to unlock the full potential of individual wheel control.This means that the user can enable or disable manual control of each wheel independently, allowing for more precise movements and adjustments.
 * __self.rot_with_pwm__ : This variable is used for rot in place in this code. It basically provides PID to the velocity of wheels.it various with the axes(given in line 112, self.rot_with_pwm = msg.axes[3]) and can vary between 1 and -1.
 * __self.steering_complete__ : This is just a boolean variable which is given as condition in drive function. This implifies if the rotation work has been done. In the end steering function, this is stated true if steering work is completed and now moving part comes in picture(drive function).
 * __self.d_arr__ : This is a list used for increment and decrement of the rate of movement in drive function. this is multiplied with the given axis speed using mode variable.


#### enc_Callback function() ->
 * The enc_callback function in the code is responsible for processing encoder data received from the rover's wheels.
 * It receives Encoder Data and updates the current encoder readings (enc_data) with the new data.

#### joycallback()->
 * The modeupbtn and modednbtn variables serve as indices in the d_arr array. As the index increases, the corresponding PWM (Pulse Width Modulation) value also increases. Essentially, the code adjusts the self.mode.
  variable based on user interactions, incrementing or decrementing it to change the rover's speed.
 * It is responsible for handling incoming messages on a specific topic. When a message is received, the callback function processes it, updates internal state and variables There are actually two modes to run a rover. One is Steering and other is Full Potential Depending the incoming message. 
 
#### steering()->

![full_potential (2)](https://github.com/user-attachments/assets/7f1a39bf-8320-43fc-9c9d-2c808e805d08)

 > for all these corresponding conditions, data from joycallback is used.
#### if both steering and full potential is locked (this aquires data from joycallback under this same condition) and
  * if forward button is pressed then wheels steer to absolute 0 degree position, since mode==1.
  * if parallel button is pressed then wheels are turned to a 90 degree absolute position as mode==1.
  * if rotinplace button is pressed then the wheels get allinged in a manner given below in the image so that and it moves in such a direction that the whole rover rotates in its same place
    
    ![Screenshot 2024-07-16 013248](https://github.com/user-attachments/assets/9fe5623b-bbd8-4e8a-b9f4-8fb414ee863a)

  *  Then velocity temp is given and the rover rotates with temp at the same place.
#### if steering is unlocked and fullpotential is locked
* and forward button is pressed then we get the current wheel angle from encode callback and use it as initial angle and then turn it 45 degrees relatively clockwise w.r.t to initial angle since mode==0.
* if parallel button is pressed same happens except in anticlockwise.
* if samedir axis value isnt 0 and oppdir value is less than threshold then all wheels rotate with same direction and speed.
* if oppdir value isnt 0 and samedir < threshold then front wheels and back wheels rotate in opposite direction.
#### if steering is locked and full potential unlocked
* we can control each wheel individually with different speeds based on the value of each wheel axes.

#### steer function() -> 
* It has 2 modes, mode = 0(for relative steer) and mode = 1(for absolute steer,doesnt depend upon initial angle value and steers to final angle value)
the **pwm** values for each of the 4 wheels is decided by the proportional factor in PID.

#### drive()->
![full_potential](https://github.com/user-attachments/assets/5fe7f1fa-ae5d-49f0-8744-49f7dd22d148)

If steering complete is true and both steering and full potential are locked :
* If rot in place is true -> It means that the rotinplace button is pressed, and from the steering() function for the same above conditions we infer that in rotinplace = true the wheels have been aligned as 45,-45,-45,45 to rotate in place, so it rotates CW or CCW based on left or right values from drive_ctrl.
* If rot in place not true -> then both velocity and angular velocity is used for normal control.if both the vel and ang_vel ques are full then avg_vel is calculated then current vel value is added to que and one value from que is removed if que is full.
* To control left right direction , all wheels move with same velocity but depending on omega -> if omega is 0 then all move straight. if u give omega to the left side wheels then rover turns right and vice versa.
