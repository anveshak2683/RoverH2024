from rscp.message.broker import Broker
import io
import rscp.message.types as message_types
import rscp_1
 
import serial

class rover_broker():
    def __init__(self) -> None:
        self.tx_buffer = bytearray()
        self.rx_buffer = bytearray()

    def rover_on_receive(self):
        pass

    def rover_on_send(self):

        sender=rscp_1.Sender()
        self.frame=sender.setparameters_frame()
        self.ser1.write(self.frame)
        self.frame=sender.setstage_frame(1)
        self.ser1.write(self.frame)
        self.frame=sender.armdis_frame(False)
        self.ser1.write(self.frame)
        self.frame=sender.navigation_to_gps_frame(12.9913113,80.2316425)
        self.ser1.write(self.frame)

    def main(self):
        # self.ser = serial.Serial("/dev/pts/4", 9600, timeout=None)
        self.ser1 = serial.Serial("/dev/ttyUSB0", 115200, timeout=None)

        self.rover_on_send()
        while True:
        # Read data from the COM port with a timeout
            data = self.ser1.read()  
            if data:
                print(f"Received: {data}")
            else:
                print("No data received within the timeout period.")
#         messages = [
#             b'~\x00\x00\xc3\t',
# b'~\x01\x01\x00\xe4.',
# b'~\t\t@I\x0eVgreen,+',
# b'~\x02\x08?\x80\x00\x00@\x00\x00\x00E\xe6',
# b'~\x03\t@I\x0eVgreen.\x16']
    
#         def on_receive(message):
#             print("Received:", message)

#         def on_send(message):
#             print("Sent:", message)

        # # Create brokers for sender and receiver
        # sender_broker = Broker(on_receive=on_receive, on_send=on_send)
        # receiver_broker = Broker(on_receive=on_receive, on_send=on_send)

        # # Create connected IO streams
        # connected_io = ConnectedIO()

        # # Dispatch messages from sender
        # for message in messages:
        #     sender_broker.dispatch(message)
        
        # # Process the messages in a loop
        # for _ in range(len(messages)):
        #     sender_broker.process(connected_io.endpoint1)
        #     receiver_broker.process(connected_io.endpoint2)
        #     connected_io.handle()

if __name__=="__main__":
    hi=rover_broker()
    hi.main()
