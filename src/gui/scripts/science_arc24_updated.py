import tkinter as tk
from tkinter import ttk
import rospy
from std_msgs.msg import Float32MultiArray,Int32MultiArray, MultiArrayLayout, MultiArrayDimension
from sensor_msgs.msg import NavSatFix
import threading

class ReadFromROS:

    def __init__(self):
        rospy.init_node('my_node')
        self.rate = rospy.Rate(10)
        self.sensordatatodisplay = [0,0,0,0,0,0,0,0,0]
        self.sendtosciencedata = [0,0,0,0]
        rospy.Subscriber('/sensor_data', Float32MultiArray, self.roscallback1)
        rospy.Subscriber('/gps_coordinates', NavSatFix, self.GPScallback)
        self.senddatascience=rospy.Publisher('/motor_data_science',Int32MultiArray,queue_size=10)
        self.xpixels = 1200
        self.ypixels = 250
        self.sensorscale = 1200/675
        self.gpsdatalat = 0
        self.gpsdatalon = 2
        self.gpsaltitud = 4

    def roscallback1(self, msg):
        self.sensordatatodisplay = msg.data  # Update the array with data from ROS message

    def tkinterinit(self, master):
        self.master = master
        self.rundisplay()  # Initialize the display
        self.getcommandsfromdisplay()
        rospy_thread = threading.Thread(target=self.spin)
        rospy_thread.start()  # Start ROS spin in a separate thread

        # Start tkinter main loop
        self.master.mainloop()

    def spin(self):
        while not rospy.is_shutdown():
            # Update tkinter labels with latest data
            self.update_labels()
            
            self.rate.sleep()

    def update_labels(self):
        # Update tkinter labels with the latest data from arrvar
        self.temp_air.set("  : " + str(self.sensordatatodisplay[6]) + " C")
        self.hum_air.set("  : " + str(self.sensordatatodisplay[1]) + " %")
        self.p_air.set("  : " + str(self.sensordatatodisplay[4]) + " hPa")        
        self.bar_alti.set("  : " + str(self.sensordatatodisplay[5]) + " m")

        self.soil_temp_1.set("  : " + str(self.sensordatatodisplay[7]) + " C")
        self.soil_temp_2.set("  : " + str(self.sensordatatodisplay[8]) + " C")
        self.soil_hum_1.set("  : " + str(self.sensordatatodisplay[2]) + " %")
        self.soil_hum_2.set("  : " + str(self.sensordatatodisplay[3]) + " %")

    def rundisplay(self):
        self.atmosphereheading  = tk.Label(self.master, text="ATMOSPHERIC CONDITIONS")
        self.temperatureair = tk.Label(self.master, text="Temperature")
        self.pressureair    = tk.Label(self.master, text="Pressure")
        self.humidityair    = tk.Label(self.master, text="Humidity")
        self.elevation      = tk.Label(self.master, text="Elevation")

        self.soilconditions1 = tk.Label(self.master, text="SOIL CONDITIONS 1" )
        self.soilconditions2 = tk.Label(self.master, text="SOIL CONDITIONS 2" )
        self.soiltemp1      = tk.Label(self.master, text="Soil Temp 1")
        self.soiltemp2      = tk.Label(self.master, text="Soil Temp 2")
        self.soilmoisture1  = tk.Label(self.master, text="Soil Moisture 1")
        self.soilmoisture2  = tk.Label(self.master, text="Soil Moisture 2")

        self.gps            = tk.Label(self.master, text="GPS" )
        self.gps_latitude   = tk.Label(self.master, text="Latitude" )
        self.gps_longitude  = tk.Label(self.master, text="Longitude" )

        self.temp_air = tk.StringVar()
        self.p_air = tk.StringVar()
        self.hum_air = tk.StringVar()
        self.bar_alti = tk.StringVar()

        self.soil_temp_1 = tk.StringVar()
        self.soil_temp_2 = tk.StringVar()
        self.soil_hum_1 = tk.StringVar()
        self.soil_hum_2 = tk.StringVar()

        self.gps_lon = tk.StringVar()
        self.gps_lat = tk.StringVar()

        self.temp_air_val = tk.Label(self.master, textvariable=self.temp_air)
        self.p_val = tk.Label(self.master, textvariable=self.p_air)
        self.hum_val = tk.Label(self.master, textvariable=self.hum_air)
        self.elev_val = tk.Label(self.master, textvariable=self.bar_alti)
        self.soil_temp_val = tk.Label(self.master, textvariable=self.soil_temp_1)
        self.soil_temp_val_2 = tk.Label(self.master, textvariable=self.soil_temp_2)
        self.mois_val = tk.Label(self.master, textvariable=self.soil_hum_1)
        self.mois_val_2 = tk.Label(self.master, textvariable=self.soil_hum_2)
        self.lat_val = tk.Label(self.master, textvariable=self.gps_lat)
        self.lon_val = tk.Label(self.master, textvariable=self.gps_lon)

        #atm part of code

        self.atmosphereheading.place(x=165,y=40,anchor='n')
        ttk.Separator(root,orient='horizontal').place(x=70, y=60, width=190)
        self.temperatureair.place(x=40,y=80)
        self.pressureair.place(x=40,y=110)
        self.humidityair.place(x=40,y=140)
        self.elevation.place(x=40,y=170)

        self.temp_air_val.place(x=180,y=80)
        self.p_val.place(x=180,y=110)
        self.hum_val.place(x=180,y=140)
        self.elev_val.place(x=180,y=170)

        ttk.Separator(root,orient='horizontal').place(x=15, y=25, width=290)
        ttk.Separator(root,orient='vertical').place(x=15, y=25, height=175)
        ttk.Separator(root,orient='horizontal').place(x=15, y=200, width=290)
        ttk.Separator(root,orient='vertical').place(x=305, y=25, height=175)

        #soil part of code

        self.soilconditions1.place(x=165,y=220,anchor='n')
        ttk.Separator(root,orient='horizontal').place(x=100, y=240, width=130)
        self.soiltemp1.place(x=40,y=260)
        self.soilmoisture1.place(x=40,y=290)

        self.soilconditions2.place(x=165,y=350,anchor='n')
        ttk.Separator(root,orient='horizontal').place(x=100, y=370, width=130)
        self.soiltemp2.place(x=40,y=390)        
        self.soilmoisture2.place(x=40,y=420)

        self.soil_temp_val.place(x=180,y=260)
        self.mois_val.place(x=180,y=290)
        self.soil_temp_val_2.place(x=180,y=390)
        self.mois_val_2.place(x=180,y=420)

        ttk.Separator(root,orient='horizontal').place(x=15, y=210, width=290)
        ttk.Separator(root,orient='vertical').place(x=15, y=210, height=120)
        ttk.Separator(root,orient='horizontal').place(x=15, y=330, width=290)
        ttk.Separator(root,orient='vertical').place(x=305, y=210, height=120)

        ttk.Separator(root,orient='horizontal').place(x=15, y=340, width=290)
        ttk.Separator(root,orient='vertical').place(x=15, y=340, height=120)
        ttk.Separator(root,orient='horizontal').place(x=15, y=460, width=290)
        ttk.Separator(root,orient='vertical').place(x=305, y=340, height=120)

        #get sensor data button
        self.get_sensor_data = tk.Button(self.master, text = "Get sensor data", command = lambda: self.sensor_cb(1), width = 10)

        self.get_sensor_data.place(x=160,y=467.5,anchor='n')

        #gps part of code
        gps_y_offset = 35

        self.gps.place(x=100,y=485 + gps_y_offset,anchor='n')
        ttk.Separator(root,orient='horizontal').place(x=85, y=505 + gps_y_offset, width=30)
        self.GPS1  = tk.Button(self.master, text="Get recent GPS",command=self.GPScallbackread,width=10)
        self.GPS1.place(x=200,y=480 + gps_y_offset,anchor='n')
        self.gps_latitude.place(x=40,y=520 + gps_y_offset)
        self.gps_longitude.place(x=40,y=550 + gps_y_offset)

        self.lat_val.place(x=180,y=520 + gps_y_offset)
        self.lon_val.place(x=180,y=550 + gps_y_offset)

        ttk.Separator(root,orient='horizontal').place(x=15, y=470 + gps_y_offset, width=290)
        ttk.Separator(root,orient='vertical').place(x=15, y=470 + gps_y_offset, height=120)
        ttk.Separator(root,orient='horizontal').place(x=15, y=590 + gps_y_offset, width=290)
        ttk.Separator(root,orient='vertical').place(x=305, y=470 + gps_y_offset, height=120)

    def getcommandsfromdisplay(self):

        self.offsertx = -310
        self.offserty = 280

        self.soilbox1 = tk.Label(self.master, text="COLLECTION BOX 1")
        self.soilbox2 = tk.Label(self.master, text="COLLECTION BOX 2")
        self.soilbox3 = tk.Label(self.master, text="COLLECTION BOX 3")
        
        self.run_pump1_8s   = tk.Button(self.master, text="PUMP 1 8s",command=lambda: self.pumpcallback(1),width=10)
        self.run_pump2_8s    = tk.Button(self.master, text="PUMP 2 8s",command=lambda: self.pumpcallback(2),width=10)

        self.run_pump1_2s    = tk.Button(self.master, text="PUMP 1 2s",command=lambda: self.pumpcallback(3),width=10)
        self.run_pump2_2s    = tk.Button(self.master, text="PUMP 2 2s",command=lambda: self.pumpcallback(4),width=10)

        self.runmotorspectro11  = tk.Button(self.master, text="SPECTRO 1",command=lambda: self.speccallback(1),width=10)
        self.runmotorspectro12  = tk.Button(self.master, text="SPECTRO 2",command=lambda: self.speccallback(2),width=10)


        self.run_lid_motor   = tk.Button(self.master, text="MOVE LID",command=lambda: self.motorcallback(1),width=10)

        ttk.Separator(root,orient='horizontal').place(x=325+self.offsertx, y=355+self.offserty, width=140)
        ttk.Separator(root,orient='horizontal').place(x=325+self.offsertx, y=710+self.offserty, width=140)
        ttk.Separator(root,orient='horizontal').place(x=475+self.offsertx, y=355+self.offserty, width=140)
        ttk.Separator(root,orient='horizontal').place(x=475+self.offsertx, y=710+self.offserty, width=140)

        ttk.Separator(root,orient='vertical').place(x=325+self.offsertx, y=355+self.offserty, height=355)
        ttk.Separator(root,orient='vertical').place(x=465+self.offsertx, y=355+self.offserty, height=355)
        ttk.Separator(root,orient='vertical').place(x=615+self.offsertx, y=355+self.offserty, height=355)
        ttk.Separator(root,orient='vertical').place(x=475+self.offsertx, y=355+self.offserty, height=355)

        self.soilbox1.place(x=395+self.offsertx,y=370+self.offserty,anchor='n')
        self.soilbox2.place(x=545+self.offsertx,y=370+self.offserty,anchor='n')

        self.run_pump1_8s.place(x=395+self.offsertx,y=410+self.offserty,anchor='n')
        self.run_pump2_8s.place(x=395+self.offsertx,y=460+self.offserty,anchor='n')
        self.run_pump1_2s.place(x=395+self.offsertx,y=510+self.offserty,anchor='n')
        self.run_pump2_2s.place(x=395+self.offsertx,y=560+self.offserty,anchor='n')
        self.runmotorspectro11.place(x=395+self.offsertx,y=610+self.offserty,anchor='n')
        self.runmotorspectro12.place(x=395+self.offsertx,y=660+self.offserty,anchor='n')

        self.run_lid_motor.place(x=545+self.offsertx,y=410+self.offserty,anchor='n')

    def motorcallback(self,a):

        msg = Int32MultiArray()
        self.sendtosciencedata[0] = a
        self.sendtosciencedata[1] = 0
        self.sendtosciencedata[2] = 0
        self.sendtosciencedata[3] = 0
        msg.data = self.sendtosciencedata
        msg.layout = MultiArrayLayout()
        msg.layout.data_offset = 0
        
        dim = MultiArrayDimension()
        dim.size = len(msg.data)
        dim.stride = len(msg.data)
        dim.label = 'write'
        msg.layout.dim = [dim]
        
        self.senddatascience.publish(msg)

    def pumpcallback(self,a):
        msg = Int32MultiArray()
        self.sendtosciencedata[1] = a
        self.sendtosciencedata[0] = 0
        self.sendtosciencedata[2] = 0
        self.sendtosciencedata[3] = 0
        msg.data = self.sendtosciencedata
        msg.layout = MultiArrayLayout()
        msg.layout.data_offset = 0
        
        dim = MultiArrayDimension()
        dim.size = len(msg.data)
        dim.stride = len(msg.data)
        dim.label = 'write'
        msg.layout.dim = [dim]
        
        self.senddatascience.publish(msg)

    def speccallback(self,a):
        msg = Int32MultiArray()
        self.sendtosciencedata[2] = a
        self.sendtosciencedata[1] = 0
        self.sendtosciencedata[0] = 0
        self.sendtosciencedata[3] = 0
        msg.data = self.sendtosciencedata
        msg.layout = MultiArrayLayout()
        msg.layout.data_offset = 0
        
        dim = MultiArrayDimension()
        dim.size = len(msg.data)
        dim.stride = len(msg.data)
        dim.label = 'write'
        msg.layout.dim = [dim]
        
        self.senddatascience.publish(msg)

    def sensor_cb(self,a):
        msg = Int32MultiArray()
        self.sendtosciencedata[0] = 0
        self.sendtosciencedata[1] = 0
        self.sendtosciencedata[2] = 0
        self.sendtosciencedata[3] = a
        msg.data = self.sendtosciencedata
        msg.layout = MultiArrayLayout()
        msg.layout.data_offset = 0
        
        dim = MultiArrayDimension()
        dim.size = len(msg.data)
        dim.stride = len(msg.data)
        dim.label = 'write'
        msg.layout.dim = [dim]
        
        self.senddatascience.publish(msg)

    def GPScallback(self,msg):
        self.gpsdatalat = msg.latitude
        self.gpsdatalon = msg.longitude

    def GPScallbackread(self):
        self.gps_lat.set(":  " + str(self.gpsdatalat))
        self.gps_lon.set(":  " + str(self.gpsdatalon))
        
if __name__ == '__main__':
    root = tk.Tk()
    root.geometry("320x1000")  # Set initial window size
    #root.resizable(False,False)
    k = ReadFromROS()
    k.tkinterinit(root)
