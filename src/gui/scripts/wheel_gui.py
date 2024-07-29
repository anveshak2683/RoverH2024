#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32 
from turtlesim.msg import Pose 
import tkinter as tk
import math
from turtlesim.srv import Spawn

class RoverWheelPositionGUI:
    def __init__(self, master):
        self.master = master
        master.title("Rover Wheel Position")

        self.label = tk.Label(self.master, text="Wheel Position: ")
        self.label.pack()

        self.position_var = tk.StringVar()
        self.position_label = tk.Label(self.master, textvariable=self.position_var)
        self.position_label.pack()

        self.canvas = tk.Canvas(self.master, width=200, height=200)
        self.canvas.pack()

        
        self.wheel = self.canvas.create_oval(50, 50, 150, 150, outline="black", width=2)
        self.center = (100, 100)
        

        rospy.init_node('wheel_position_gui', anonymous=True)
        rospy.Subscriber('/turtle1/pose', Pose, callback=self.callback)
        
        
    def call_spawn_service(self,name): 
        self.name = name
        try:
            spawn = rospy.ServiceProxy("/spawn", Spawn)
            response= spawn(8,5.44, 0, self.name)
        except rospy.ServiceException as e:
            rospy.logwarn(e)

    

    def callback(self, pose=Pose):
        position = round(pose.theta,2)
        self.position_var.set(f"Position: {position}")
        self.rotate(position)

    def rotate(self, angle):
        
        self.canvas.delete("all")
        x,y = self.center
        radius = 50
        angle_rad = math.radians(angle * 180 / math.pi)  
        end1_x = x + radius * math.cos(angle_rad)
        end1_y = y - radius * math.sin(angle_rad)
        end2_x = x - radius * math.cos(angle_rad)
        end2_y = y + radius * math.sin(angle_rad)
        
        self.canvas.create_line(end2_x, end2_y, end1_x, end1_y, fill="red", width=2)

if __name__ == "__main__":
    root = tk.Tk()
    gui = RoverWheelPositionGUI(root)
    root.mainloop()
