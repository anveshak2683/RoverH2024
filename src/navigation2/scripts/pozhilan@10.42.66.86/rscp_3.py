from rscp.message.broker import Broker
import io
import rscp.message.types as message_types
from rscp_2 import receiver
import rscp_1
import serial

class rover_broker():
    def __init__(self) -> None:
        self.ser1 = serial.Serial("/dev/ttyUSB0", 115200, timeout=None)  
        self.RSCP_Object = receiver(self.ser1)
            
    def rover_on_receive(self):
        pass

    def rover_on_send(self):

        sender=rscp_1.Sender()
        self.frame=sender.setparameters_frame({"frame":1,
            "latitude":12.911023,
            "longitude":80.2220012,
            "text": "Pranav",
            "array":[1,2]})
        self.ser1.write(self.frame)
        self.frame=sender.setstage_frame(1)
        self.ser1.write(self.frame)
        self.frame=sender.armdis_frame(False)
        self.ser1.write(self.frame)
        self.frame=sender.navigation_to_gps_frame(12.911022,80.222003)
        self.ser1.write(self.frame)

    def main(self):
        # self.ser = serial.Serial("/dev/pts/4", 9600, timeout=None)
        self.rover_on_send()
        while True:
        # Read data from the COM port with a timeout
            data = self.ser1.read()  
            if data:
                self.RSCP_Object.test_parser(data)
                print(data)
            else:
                print("No data received within the timeout period.")
if __name__=="__main__":
    hi=rover_broker()
    hi.main()
