#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32MultiArray,Float32MultiArray
import tkinter as tk
import math

class RoverWheelPositionGUI:
    def __init__(self, master):
        self.master = master
        master.title("Rover Wheel Position")
        self.cp_arrow=[['grey','none']]*4

        #self.label = tk.Label(self.master, text="Wheel Position: ")
        #self.label.pack()
        self.canvas = tk.Canvas(self.master, width=400, height=400, bg='white')
        self.canvas.grid(row=0, column=0, sticky="nsew")
        self.canvas.pack(fill=tk.BOTH, expand=True)

        rospy.init_node('wheel_position_gui', anonymous=True)
        rospy.Subscriber('/enc_auto', Float32MultiArray, self.enc_callback)
        rospy.Subscriber('/motor_pwm', Int32MultiArray, self.pwm_callback)
        rospy.Subscriber('/gui_msgs', Int32MultiArray, self.gui_callback)


        self.canvas.create_line(0,650,700,650,fill='black',width=5)
        self.canvas.create_line(700,0,700,650,fill='black',width=5)
        self.canvas.create_line(130,150,500,150,fill='black',width=5,capstyle='round')
        self.canvas.create_line(130,150,130,450,fill='black',width=5,capstyle='round')
        self.canvas.create_line(500,150,500,450,fill='black',width=5,capstyle='round')
        self.canvas.create_line(130,450,500,450,fill='black',width=5,capstyle='round')
        # self.canvas.create_line(197,125,197,475,fill='black',width=5,capstyle='round')
        # self.canvas.create_line(432,125,432,475,fill='black',width=5,capstyle='round')
        # self.canvas.create_line(197,125,432,125,fill='black',width=5,capstyle='round')
        # self.canvas.create_line(197,475,432,475,fill='black',width=5,capstyle='round')
  
        self.position_vars = []
        self.position_labels = []
        self.wheel_dir = []
        self.positions = [(50, 20), (450, 20), (50, 320), (450, 320)]
        for pos in self.positions:
            pos_var = tk.StringVar()
            pos_label = tk.Label(self.master, textvariable=pos_var)
            pos_label.place(x=pos[0], y=pos[1])
            self.position_vars.append(pos_var)
            self.position_labels.append(pos_label)

        self.center_positions = [(130, 150), (500, 150), (130, 450), (500, 450)]
        self.wheel_lines = []
        self.black_boxes = []
        self.color_arrow = 0
        self.pos_arrow = 0
        for i in range(4):
            center = self.center_positions[i]
            black_box = self.canvas.create_line(center[0], center[1], center[0], center[1], fill="grey", width=50)
            self.black_boxes.append(black_box)

            wheel_line = self.canvas.create_line(center[0], center[1], center[0], center[1], fill="black", width=45, capstyle='round')
            self.wheel_lines.append(wheel_line)

            wheel_dir = self.canvas.create_line(center[0], center[1], center[0], center[1], fill=self.cp_arrow[i][0], width=5, arrow=self.cp_arrow[i][1], arrowshape=(25,25,10))
            self.wheel_dir.append(wheel_dir)

        self.speed_var = tk.DoubleVar()
        self.mode_var = tk.IntVar()
        


    def enc_callback(self, msg):
        self.list = [msg.data[0], msg.data[3], msg.data[2], msg.data[5]]
        self.wheel()


    def pwm_callback(self,msgs):
        self.list2 = msgs.data
        self.arrow()

    def gui_callback(self,guimsg):
        self.list3 = guimsg.data
        # if guimsg.data:
        #     self.mode_var.set(guimsg.data[0])
        # else:
        #     self.mode_var.set(0)
        self.speed_var.set(guimsg.data[16])
        self.speed()
        self.mode(guimsg)

    def mode(self,guimsg):
        self.mode_var.set(guimsg.data[0])
        mode = self.mode_var.get()
        colour=['white']
        if mode== 0:
            colour[0]='#ff0000'
        elif mode ==1:
            colour[0]='#bf3f00'
        elif mode ==2:
            colour[0]='#7f7f00'
        elif mode ==3:
            colour[0]='#fFbf00'
        elif mode ==4:
            colour[0]='#00ff00'
        self.canvas.create_oval(1150, 50, 1450, 350, outline="black", fill=colour,width=12)


    def speed(self):
        for i in range(-90, 91, 10):
            angle_ = math.radians(180 - i) - (math.pi/2)
            x0 = 1300 + 140 * math.cos(angle_)
            y0 = 200 - 140 * math.sin(angle_)
            x1 = 1300 + 160 * math.cos(angle_)
            y1 = 200 - 160 * math.sin(angle_)
            self.canvas.create_line(x0, y0, x1, y1, fill="black", width=2)
            
            speed = int(i / 90 * 110)
            x_text = 1300 + 120 * math.cos(angle_)
            y_text = 200 - 120 * math.sin(angle_)
            self.canvas.create_text(x_text, y_text, text=str(speed), font=("Helvetica", 10))
        

        self.needle = self.canvas.create_line(1300, 200, 1300, 60, fill="blue",width=5)
        self.speed_run()
    def speed_run(self):
        speed = self.speed_var.get()

        angle = 90 - (speed / 110 * 90)
        angle_rad = math.radians(angle)
        
        x = 1300 + 140 * math.cos(angle_rad)
        y = 200 - 140 * math.sin(angle_rad)

        self.canvas.coords(self.needle, 1300, 200, x, y)
        
        # Update the GUI every 100 ms
        self.master.after(100, self.speed_run)
        
    
    def arrow(self):
        for i in range(4):
            if self.list2[i] < 0:
                self.cp_arrow[i] = ['red', 'first']
            elif self.list2[i] > 0:
                self.cp_arrow[i] = ['green', 'last']
            else:
                self.cp_arrow[i] = ['grey', 'none']

        
    def wheel(self):
        for i in range(4):
            position = round(self.list[i], 2)
            self.position_vars[i].set(f"Position: {position}")
            self.rotate(position, self.center_positions[i], i)
        

    def rotate(self, angle, center, index):
        x, y = center
        radius = 70
        angle_rad = (-angle + 90)*(math.pi/180)
         
        end1_x = x + radius * math.cos(angle_rad)
        end1_y = y - radius * math.sin(angle_rad)
        end2_x = x - radius * math.cos(angle_rad)
        end2_y = y + radius * math.sin(angle_rad)
        end3_x = x + 65 * math.cos(-angle*(math.pi/180))
        end3_y = y - 65 * math.sin(-angle*(math.pi/180))
        end4_x = x - 65 * math.cos(-angle*(math.pi/180))
        end4_y = y + 65 * math.sin(-angle*(math.pi/180))

        if center[0] == 500:  # for the black box being inward for 2nd and 4th wheels
            self.canvas.coords(self.black_boxes[index], x, y, end4_x, end4_y)
        else:
            self.canvas.coords(self.black_boxes[index], x, y, end3_x, end3_y)
        
        self.canvas.coords(self.wheel_lines[index], end2_x, end2_y, end1_x, end1_y)

        self.canvas.itemconfig(self.wheel_dir[index], fill=self.cp_arrow[index][0], arrow=self.cp_arrow[index][1])
        self.canvas.coords(self.wheel_dir[index], end2_x, end2_y, end1_x, end1_y)


if __name__ == "__main__":
    root = tk.Tk()
    gui = RoverWheelPositionGUI(root)
    root.mainloop()

