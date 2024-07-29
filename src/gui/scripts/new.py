#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32MultiArray, Float32MultiArray
import tkinter as tk
import math
from tkinter import ttk

class RoverWheelPositionGUI:
    def __init__(self, master):
        self.master = master
        master.title("Rover Wheel Position")
        self.cp_arrow = [['grey', 'none'],['grey', 'none'],['grey', 'none'],['grey', 'none']]

        self.canvas = tk.Canvas(self.master, width=1400, height=400, bg='white')
        self.canvas.grid(row=0, column=0, sticky="nsew")
        self.canvas.pack(fill=tk.BOTH, expand=True)

        rospy.init_node('wheel_position_gui', anonymous=True)
        rospy.Subscriber('/enc_auto', Float32MultiArray, self.enc_callback)
        rospy.Subscriber('/motor_pwm', Int32MultiArray, self.pwm_callback)
        rospy.Subscriber('/gui_msgs', Int32MultiArray, self.gui_callback)

        
        self.draw_background()

        self.init_gui_elements()

    def draw_background(self):
        #ttk.Separator(self.master,orient='horizontal').place(x=0,y=650,relwidth=0.2)
        self.canvas.create_line(0,650,2000,650,fill='black',width=5)
        self.canvas.create_line(1000,0,1000,650,fill='black',width=5)
        self.canvas.create_line(300,170,670,170,fill='black',width=5,capstyle='round')
        self.canvas.create_line(300,170,300,470,fill='black',width=5,capstyle='round')
        self.canvas.create_line(670,170,670,470,fill='black',width=5,capstyle='round')
        self.canvas.create_line(300,470,670,470,fill='black',width=5,capstyle='round')

        self.speed_oval = self.canvas.create_oval(1300, 220, 1600, 520, outline="black", fill="white", width=12)

        self.needle = self.canvas.create_line(1450, 370, 1450, 230, fill="blue", width=5)

        for i in range(-90, 91, 10):
            angle_rad = math.radians(180 - i) - (math.pi/2)
            x0 = 1450 + 140 * math.cos(angle_rad)
            y0 = 370 - 140 * math.sin(angle_rad)
            x1 = 1450 + 160 * math.cos(angle_rad)
            y1 = 370 - 160 * math.sin(angle_rad)
            self.canvas.create_line(x0, y0, x1, y1, fill="black", width=2)
            
            speed = int(i / 90 * 110)
            x_text = 1450 + 120 * math.cos(angle_rad)
            y_text = 370 - 120 * math.sin(angle_rad)
            self.canvas.create_text(x_text, y_text, text=str(speed), font=("Helvetica", 10))


    def init_gui_elements(self):
        self.position_vars = []
        self.position_labels = []
        self.positions = [(220, 40), (620, 40), (220, 340), (620, 340)]
        for pos in self.positions:
            pos_var = tk.StringVar()
            pos_label = tk.Label(self.master, textvariable=pos_var)
            pos_label.place(x=pos[0], y=pos[1])
            self.position_vars.append(pos_var)
            self.position_labels.append(pos_label)

        self.center_positions = [(300, 170), (670, 170), (300, 470), (670, 470)]
        self.wheel_lines = []
        self.black_boxes = []
        self.wheel_dir = []
        for i in range(4):
            center = self.center_positions[i]

            black_box = self.canvas.create_line(center[0], center[1], center[0], center[1], fill="grey", width=50)
            self.black_boxes.append(black_box)

            wheel_line = self.canvas.create_line(center[0], center[1], center[0], center[1], fill="black", width=45, capstyle='round')
            self.wheel_lines.append(wheel_line)

            wheel_dir = self.canvas.create_line(center[0], center[1], center[0], center[1], fill=self.cp_arrow[i][0], width=5, arrow=self.cp_arrow[i][1], arrowshape=(25, 25, 10))
            self.wheel_dir.append(wheel_dir)
        
        self.speed_var = tk.DoubleVar()
        self.mode_var = tk.StringVar()
        self.mode_label = tk.Label(self.master, textvariable=self.mode_var, font=("Times New Roman", 12), fg='white',bg='grey25')
        self.mode_label.place(x=1450, y=100,anchor='center')
        self.lock_var = tk.StringVar()
        self.lock_label = tk.Label(self.master, textvariable=self.lock_var, font=("Times New Roman", 32), fg='white',bg='black')
        self.lock_label.place(x=900, y=740,anchor='center')
        self.button_var = tk.StringVar()
        self.button_label = tk.Label(self.master, textvariable=self.button_var, font=("Times New Roman", 32), fg='white',bg='black')
        self.button_label.place(x=900, y=900,anchor='center')
        self.diff_var = tk.StringVar()
        self.diff_label = tk.Label(self.master, textvariable=self.diff_var, font=("Times New Roman", 32), fg='white',bg='black')
        self.diff_label.place(x=900, y=1000,anchor='center')

    def enc_callback(self, msg):
        self.list = [msg.data[0], -msg.data[3], -msg.data[2], msg.data[5]]
        self.wheel()

    def pwm_callback(self, msgs):
        self.list2 = [-msgs.data[0],-msgs.data[1],-msgs.data[2],-msgs.data[3]]
        self.arrow()

    def gui_callback(self, guimsg):
        self.list3 = guimsg.data
        self.mode(guimsg)
        self.speed_var.set(guimsg.data[16])
        self.speed()
        self.lock_unlock(guimsg)
        self.buttons(guimsg)
        self.differential(guimsg)

    def differential(self,guimsg):
        drive_diff=guimsg.data[17]
        if drive_diff!=0 and self.list2[0]*self.list2[1] < 0 and self.list2[2]*self.list2[3] < 0:
            diff='Differential'
        else:
            diff=''                           
        self.diff_var.set(diff)             

    def mode(self, guimsg):
        mode = guimsg.data[0]
        colour = ['white']
        font_size = [12]
        if mode == 0:
            colour[0] = 'lawn green'
            font_size[0] = 16
        elif mode == 1:
            colour[0] = '#dfff00'
            font_size[0] = 20
        elif mode == 2:
            colour[0] = '#eebf22'
            font_size[0] = 24
        elif mode == 3:
            colour[0] = '#ff7f24'
            font_size[0] = 28
        elif mode == 4:
            colour[0] = '#ee2c2c'
            font_size[0] = 36
            #self.blink_mode_label()
        
        self.mode_var.set(f"MODE: {mode}")
        self.mode_label.config(font=("Times New Roman", font_size[0]), fg=colour[0])
        self.canvas.itemconfig(self.speed_oval, fill=colour[0])
    
    # def blink_mode_label(self):
    #     current_color = self.mode_label.cget("fg")
    #     next_color = "white" if current_color == "black" else "black"
    #     self.mode_label.config(fg=next_color)
    #     self.master.after(500, self.blink_mode_label)


    def speed(self):
        speed = self.speed_var.get()
        angle = 90 - (speed / 110 * 90)
        angle_rad = math.radians(angle)
        x = 1450 + 140 * math.cos(angle_rad)
        y = 370 - 140 * math.sin(angle_rad)
        self.canvas.coords(self.needle, 1450, 370, x, y)

    def lock_unlock(self,guimsg):
        ind_steer=guimsg.data[1]
        steer=guimsg.data[2] 
        if ind_steer == 0 and steer == 0:
            lock='                                ______________DRIVE MODE________________                           '
            colour='cyan'
        elif ind_steer==0 and steer == 1:
            lock='                                _______________STEER MODE_______________                           '
            colour='green2'
        elif ind_steer==1 and steer==0:
            lock='                              __________INDIVIDUAL STEERING MODE_________                        '
            colour='firebrick1' 
        self.lock_var.set(lock)
        self.lock_label.config(fg=colour)
    
    def buttons(self,guimsg):
        drive_x=guimsg.data[3]
        drive_y=guimsg.data[4]
        drive_b=guimsg.data[5]
        steer_y=guimsg.data[7]
        steer_b=guimsg.data[8]
        steer_fb=guimsg.data[9]
        steer_lr=guimsg.data[10]
        driver_fb=guimsg.data[11]
        driver_lr=guimsg.data[12]
        drivel_fb=guimsg.data[13]
        drivel_lr=guimsg.data[14]
        rotinplacevel=guimsg.data[15]
        rot=guimsg.data[6]
        if drive_x==1:
            button='Rotating steering forward to [0,0,0,0]'
        elif drive_y==1:
            button='Rotating steering perpendicular to rover to [90,90,90,90]'
        elif drive_b==1:
            button='Rotating steering for in place rotation to [45,-45,-45,45]'
        elif steer_y==1:
            button='Turning steering clockwise by 45 degrees'
        elif steer_b==1:
            button='Turning steering anti-clockwise by 45 degrees'
        elif steer_fb!= 0:
            button=f'All wheels ->     SAME     direction with speed     {steer_fb}    '
        elif steer_lr!= 0:
            button=f'All wheels ->       OPPOSITE      direction with speed      {steer_lr}     '
        elif driver_fb!= 0:
            button=f'Moving         FRONT LEFT        wheel with speed     {driver_fb}    '
        elif driver_lr!= 0:
            button=f'Moving         FRONT RIGHT       wheel with speed      {driver_lr}     '
        elif drivel_fb!= 0:
            button=f'Moving         BACK LEFT         wheel with speed     {drivel_fb}    '
        elif drivel_lr!= 0:
            button=f'Moving         BACK RIGHT        wheel with speed      {drivel_lr}     '
        elif rotinplacevel!=0:
            button=f'Rotation speed =      {rotinplacevel}      '
        elif rot!=0:
            button=f'Rotation in place with velocity =         {rot}         '
        else:
            button=''
        self.button_var.set(button)

    def arrow(self):
        for i in range(4):  
            if self.list2[i] < 0:
                self.cp_arrow[i] = ['red', 'first']
            elif self.list2[i] > 0:
                self.cp_arrow[i] = ['green2', 'last']
            else:
                self.cp_arrow[i] = ['grey', 'none']
            self.canvas.itemconfig(self.wheel_dir[i], fill=self.cp_arrow[i][0], arrow=self.cp_arrow[i][1])

    def wheel(self):
        for i in range(4):
            position = round(self.list[i], 2)
            self.position_vars[i].set(f"Position: {position}")
            self.rotate(position, self.center_positions[i], i)

    def rotate(self, angle, center, index):
        x, y = center
        radius = 70
        angle_rad = (-angle + 90) * (math.pi / 180)
        end1_x = x + radius * math.cos(angle_rad)
        end1_y = y - radius * math.sin(angle_rad)
        end2_x = x - radius * math.cos(angle_rad)
        end2_y = y + radius * math.sin(angle_rad)
        end3_x = x + 65 * math.cos(-angle * (math.pi / 180))
        end3_y = y - 65 * math.sin(-angle * (math.pi / 180))
        end4_x = x - 65 * math.cos(-angle * (math.pi / 180))
        end4_y = y + 65 * math.sin(-angle * (math.pi / 180))

        if center[0] == 670:  # for the black box being inward for 2nd and 4th wheels
            self.canvas.coords(self.black_boxes[index], x, y, end4_x, end4_y)
        else:
            self.canvas.coords(self.black_boxes[index], x, y, end3_x, end3_y)
        self.canvas.coords(self.wheel_lines[index], end2_x, end2_y, end1_x, end1_y)
        self.canvas.coords(self.wheel_dir[index], end2_x, end2_y, end1_x, end1_y)

if __name__ == "__main__":
    root = tk.Tk()
    gui = RoverWheelPositionGUI(root)
    root.mainloop()
