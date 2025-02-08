import rospy
from std_msgs.msg import Bool, Float32, Float32MultiArray
from tkinter import Tk, Label, Button, Frame
from tkinter.messagebox import showinfo


class ControlGUI:
    def __init__(self, master):
        self.master = master
        master.title("Robot Control Interface")
        master.geometry("700x500")
        master.configure(bg="#2a2a2a")

        # ROS Initialization
        rospy.init_node("gui_node", anonymous=True)
        self.sensor_sub = rospy.Subscriber("sensor_data", Float32MultiArray, self.sensor_callback)
        self.pump1_pub = rospy.Publisher("pump1_control", Bool, queue_size=10)
        self.pump2_pub = rospy.Publisher("pump2_control", Bool, queue_size=10)
        self.servo1_pub = rospy.Publisher("servo1_control", Float32, queue_size=10)
        self.servo2_pub = rospy.Publisher("servo2_control", Float32, queue_size=10)
        self.servo3_pub = rospy.Publisher("servo3_control", Float32, queue_size=10)
        self.stepper_pub = rospy.Publisher("stepper_control", Float32, queue_size=10)

        self.sensor_data = [None] * 9

        # Title
        title = Label(master, text="Robot Control Panel", bg="#2a2a2a", fg="white", font=("Helvetica", 18, "bold"))
        title.pack(pady=10)

        # Sensor Data Buttons
        sensor_frame = Frame(master, bg="#2a2a2a")
        sensor_frame.pack(pady=20)

        ref_button = Button(sensor_frame, text="Reference Values", bg="#4CAF50", fg="white",
                            font=("Helvetica", 12), command=self.show_reference_values)
        ref_button.grid(row=0, column=0, padx=10)

        data_button = Button(sensor_frame, text="Sensor Data", bg="#2196F3", fg="white",
                             font=("Helvetica", 12), command=self.show_sensor_data)
        data_button.grid(row=0, column=1, padx=10)

        # Motor Pump Buttons
        pump_frame = Frame(master, bg="#2a2a2a")
        pump_frame.pack(pady=20)

        pump1_label = Label(pump_frame, text="Pump 1:", bg="#2a2a2a", fg="white", font=("Helvetica", 12))
        pump1_label.grid(row=0, column=0, padx=10)
        pump1_run = Button(pump_frame, text="Run (8s)", bg="#4CAF50", fg="white",
                           font=("Helvetica", 10), command=lambda: self.run_pump(self.pump1_pub))
        pump1_run.grid(row=0, column=1, padx=5)
        pump1_stop = Button(pump_frame, text="Stop", bg="#F44336", fg="white",
                            font=("Helvetica", 10), command=lambda: self.stop_pump(self.pump1_pub))
        pump1_stop.grid(row=0, column=2, padx=5)

        pump2_label = Label(pump_frame, text="Pump 2:", bg="#2a2a2a", fg="white", font=("Helvetica", 12))
        pump2_label.grid(row=1, column=0, padx=10)
        pump2_run = Button(pump_frame, text="Run (8s)", bg="#4CAF50", fg="white",
                           font=("Helvetica", 10), command=lambda: self.run_pump(self.pump2_pub))
        pump2_run.grid(row=1, column=1, padx=5)
        pump2_stop = Button(pump_frame, text="Stop", bg="#F44336", fg="white",
                            font=("Helvetica", 10), command=lambda: self.stop_pump(self.pump2_pub))
        pump2_stop.grid(row=1, column=2, padx=5)

        # Servo Buttons
        servo_frame = Frame(master, bg="#2a2a2a")
        servo_frame.pack(pady=20)

        for i, pub in enumerate([self.servo1_pub, self.servo2_pub, self.servo3_pub], start=1):
            servo_label = Label(servo_frame, text=f"Servo {i}:", bg="#2a2a2a", fg="white", font=("Helvetica", 12))
            servo_label.grid(row=i-1, column=0, padx=10)
            servo_zero = Button(servo_frame, text="0°", bg="#2196F3", fg="white",
                                font=("Helvetica", 10), command=lambda p=pub: self.set_servo_angle(p, 0))
            servo_zero.grid(row=i-1, column=1, padx=5)
            servo_125 = Button(servo_frame, text="125°", bg="#FFC107", fg="black",
                               font=("Helvetica", 10), command=lambda p=pub: self.set_servo_angle(p, 125))
            servo_125.grid(row=i-1, column=2, padx=5)

        # Stepper Motor Control
        stepper_frame = Frame(master, bg="#2a2a2a")
        stepper_frame.pack(pady=20)

        stepper_label = Label(stepper_frame, text="Stepper Motor:", bg="#2a2a2a", fg="white", font=("Helvetica", 12))
        stepper_label.grid(row=0, column=0, padx=10)

        stepper_plus = Button(stepper_frame, text="+90°", bg="#4CAF50", fg="white",
                              font=("Helvetica", 10), command=lambda: self.set_stepper_angle(90))
        stepper_plus.grid(row=0, column=1, padx=5)

        stepper_minus = Button(stepper_frame, text="-90°", bg="#F44336", fg="white",
                               font=("Helvetica", 10), command=lambda: self.set_stepper_angle(-90))
        stepper_minus.grid(row=0, column=2, padx=5)

    def sensor_callback(self, msg):
        self.sensor_data = msg.data

    def show_reference_values(self):
        ref_values = """
        Reference Values:
        Temperature (DHT): 22-30°C
        Humidity (DHT): 40-60%
        Soil Moisture: 40-80%
        Pressure: ~1013 hPa
        """
        showinfo("Reference Values", ref_values)

    def show_sensor_data(self):
        data = "\n".join([
            f"Temperature (DHT): {self.sensor_data[0]:.2f}°C",
            f"Humidity (DHT): {self.sensor_data[1]:.2f}%",
            f"Soil Moisture A: {self.sensor_data[2]:.2f}%",
            f"Soil Moisture B: {self.sensor_data[3]:.2f}%",
            f"Pressure: {self.sensor_data[4]:.2f} hPa",
            f"Altitude: {self.sensor_data[5]:.2f} m",
            f"Temperature (BMP): {self.sensor_data[6]:.2f}°C",
            f"DS18B20 Temp 1: {self.sensor_data[7]:.2f}°C",
            f"DS18B20 Temp 2: {self.sensor_data[8]:.2f}°C"
        ]) if all(x is not None for x in self.sensor_data) else "No sensor data available."
        showinfo("Sensor Data", data)

    def run_pump(self, publisher):
        publisher.publish(Bool(data=True))
        self.master.after(8000, lambda: self.stop_pump(publisher))  # Auto-stop after 8s

    def stop_pump(self, publisher):
        publisher.publish(Bool(data=False))

    def set_servo_angle(self, publisher, angle):
        publisher.publish(Float32(data=angle))

    def set_stepper_angle(self, angle):
        self.stepper_pub.publish(Float32(data=angle))


if __name__ == "__main__":
    try:
        root = Tk()
        gui = ControlGUI(root)
        root.mainloop()
    except rospy.ROSInterruptException:
        pass

