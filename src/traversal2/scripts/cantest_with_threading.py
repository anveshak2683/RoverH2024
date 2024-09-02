import can
import threading
import time
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8, Float32, Int32MultiArray


class CAN_Class:
    def __init__(self):

        rospy.init_node("can_test")
        rospy.Subscriber("joy", Joy, self.joyCallback)
        rospy.Subscriber("test_can", Int8, self.int8callback)
        self.msg = can.Message(
            arbitration_id=0x123,
            is_extended_id=False,
            dlc=6,  
            is_fd = True,
            data= [1,2,3,4,5,6]
        )

        self.bus = can.ThreadSafeBus(channel='can0', bustype='socketcan', fd=True)
        
        self.data_lock = threading.Lock()

        self.event = threading.Event()
        receiver_thread = threading.Thread(target=self.receive_messages, args=())
        receiver_thread.daemon = True
        receiver_thread.start()
        
        sender_thread = threading.Thread(target=self.send_messages, args=())
        sender_thread.daemon = True
        sender_thread.start()
    
    def int8callback(self, msg):
        self.data_lock.acquire()
        if(self.data_lock.locked() == True):
            self.msg.arbitration_id = 0x456
            self.msg.data = [msg.data]
            self.msg.dlc = 1
            self.event.set()
            self.data_lock.release()

    def joyCallback(self,msg):
        self.data_lock.acquire()
        if(self.data_lock.locked() == True):
            self.msg.arbitration_id = 0x123
            self.msg.data =  [int(127*i+127) for i in msg.axes]
            self.msg.dlc = len(self.msg.data)
            self.event.set()
            self.data_lock.release()

    def receive_messages(self):
        while True:
            try:
                msg = self.bus.recv()
                if msg:
                    print(f"Received message: {msg}")
            except can.CanError as e:
                print(f"Error receiving message: {e}")
                break

    def send_messages(self):
        while True:
            # Create a CAN FD message
            
            try:

                event_set = self.event.wait()
                if event_set:
                    self.bus.send(self.msg)
                    self.event.clear()
                #print("Message sent successfully.")
            except can.CanError as e:
                print(f"Error sending message: {e}")
                break
            

    def mainthread(self):
        
        rate = rospy.Rate(10)
        try:
            while not rospy.is_shutdown():
                rate.sleep()  
        except KeyboardInterrupt:
            print("Interrupted by user")
        finally:
            self.bus.shutdown()

if __name__ == "__main__":
    myObject = CAN_Class()
    myObject.mainthread()

