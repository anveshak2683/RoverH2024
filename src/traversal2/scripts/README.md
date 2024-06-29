All Traversal Codes for both IRoC and ARC Rover
==============================================

* amogh_drive.py: Basic Drive Code written by Amogh and Pranav without Single steering control, it has an option for autonomous *A* button, which we can transfer to full potential later. 

* drive.py: The original code written by Pranav without any special functions. Revert to this code *ONLY* if nothing else works. 

* Full_Potential_Steering.py: Primary drive code which will most likely be used in ARC. Individual steering and PWM control for *rotinplace*.

* Full_Potential_Steering_1.py: Kavin tried to fix directions, but failed to do so. This code is NOT to be used right now until further fixes happen. 

* iroc_drive_v1.py: Primary IRoC Drive code. An adaptation of the ARC drive.py for four wheels and no steering. It contains ledc-type control for gradually increasing speed (moving average). 

* iroc_drive_v1_with_102.py: Just a small change with autonomous input following zero speed = 127, as was the case in Galileo earlier. 

* iroc_drive_tanish.py: Foxglove code; Joystick subscribing is different, i.e. the callback continues even if no commands are given, and axes are different. Use this if Foxglove GUI is used for joystick control. 

* **which_wheel.py**: This code was made to check which index of motor_pwm corresponds to which drive/steering motor, mostly to play around with the speed of any individual motor. Used to debug individual motor failures and calibrate speeds. 
Do not pay any attention to the joystick axes or to the sign of the published integers, just check which axis of motor_pwm has a number when a certain wheel moves (written by Amogh, used code written by Pranav)

