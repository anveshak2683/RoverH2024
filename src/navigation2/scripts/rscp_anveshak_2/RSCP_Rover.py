from rscp.frame_parser import Frame, FrameParser
from rscp.message import types
from typing import List, Type
import io
import rscp.message.types as message_types
import serial
import rospy
from std_msgs.msg import Bool, Float32, Int32 ,String 
from navigation.msg import auto , detection
import sys

class RSCP_Receiver:
    def __init__(self) -> None:
        rospy.init_node("pozhilan")
        self.data=None

        self.rscp_data=auto()

        self.task_completed_sub= rospy.Subscriber("/task_completed", Int32, callback= self.task_completed_callback)
        self.detection_sub= rospy.Subscriber("/detection", detection, callback= self.detection_callback)

        self.rscp_data_pub=rospy.Publisher("/rscp_data",auto,queue_size=10)
        self.check_received = False
        self.rx=bytearray()
        self.ser=serial.Serial(sys.argv[1], baudrate=115200, timeout=None)
        if self.ser.is_open:
            print(f"Connected to {self.ser.name}")

    def task_completed_callback(self,check) :
        #print("Callback activated", check.data)       
        if check.data==1 :
            # self.check_received = True
            self.frame=self.task_completed_frame()
            self.ser.write(self.frame)

    def detection_callback(self,msg : detection):

        self.frame=self.dedect_frame(detection.depth,detection.colour)
        self.ser.write(self.frame)


    def on_update(self,frame):
        self.msg_id = frame.msg_id
        self.data=frame.data
        if frame.msg_id == 0:
            self.acknowledge_body()
            self.rx=bytearray()
            

        elif frame.msg_id == 1:
            self.armdisarm_body()
            self.rx=bytearray()
            self.frame=self.ack_frame()
            self.ser.write(self.frame)

        elif frame.msg_id == 2:
            self.navigatetoGPS_body()
            self.rx=bytearray()
            self.frame=self.ack_frame()
            self.ser.write(self.frame)


        elif frame.msg_id == 3:
            self.taskfinished_body()
            self.rx=bytearray()
            self.frame=self.ack_frame()
            self.ser.write(self.frame)

        elif frame.msg_id == 4:
            self.setstage_body()
            self.rx=bytearray()
            self.frame=self.ack_frame()
            self.ser.write(self.frame)
            
        elif frame.msg_id == 5:
            self.text()
            self.rx=bytearray()
            self.frame=self.ack_frame()
            self.ser.write(self.frame)

        elif frame.msg_id == 6:
            self.arucotag_body()
            self.rx=bytearray()
            self.frame=self.ack_frame()
            self.ser.write(self.frame)

        elif frame.msg_id == 7:
            self.locatearucotags_body()
            self.rx=bytearray()
            self.frame=self.ack_frame()
            self.ser.write(self.frame)

        elif frame.msg_id == 8:
            self.locate3d_body()
            self.rx=bytearray()
            self.frame=self.ack_frame()
            self.ser.write(self.frame)

        elif frame.msg_id == 9:
            self.detection_body()
            self.rx=bytearray()
            self.frame=self.ack_frame()
            self.ser.write(self.frame)

        elif frame.msg_id == 10:
            self.setparameters_body()
            self.rx=bytearray()
            self.frame=self.ack_frame()
            self.ser.write(self.frame)



    

    def test_parser(self,bytestream):
        self.rx.extend(bytestream)
        frame_parser = FrameParser(self.on_update)

        for byte in self.rx:
            frame_parser.process(byte)




    def acknowledge_body(self):
        self.body = types.Acknowledge.deserialize(self.data)
        print(f"Body of the Message = {self.body}")
    
    def armdisarm_body(self):
        self.body=types.ArmDisarm.deserialize(self.data)
        self.rscp_data.arm=self.body.arm
        self.rscp_data.msg_id = self.msg_id
        self.rscp_data_pub.publish(self.rscp_data)
        print(f"Body of the Message = {self.body}")     
    
    def navigatetoGPS_body(self):
        self.body = types.NavigateToGPS.deserialize(self.data)
        self.rscp_data.msg_id = self.msg_id
        self.rscp_data.latitude = self.body.latitude
        self.rscp_data.longitude = self.body.longitude
        self.rscp_data_pub.publish(self.rscp_data)
        print(f"Body of the Message = {self.body}")     


    
    def taskfinished_body(self):
        self.body = types.TaskFinished.deserialize(self.data)
        print(f"Body of the Message = {self.body}")     
    
    def setstage_body(self):
        self.body = types.SetStage.deserialize(self.data)
        self.rscp_data.msg_id = self.msg_id
        self.rscp_data.setstage = self.body.stage_id
        self.rscp_data_pub.publish(self.rscp_data)
        print(f"Body of the Message = {self.body}")     

    
    def text(self):
        self.body = types.Text.deserialize(self.data)
        self.rscp_data.msg_id = self.msg_id
        self.rscp_data.text = self.body.text
        self.rscp_data_pub.publish(self.rscp_data)
        print(f"Body of the Message = {self.body}")     

    
    
    def arucotag_body(self):
        self.body=types.ArucoTag.deserialize(self.data)
        print(f"Body of the Message = {self.body}")     
        
        
        
    
    def locatearucotags_body(self):
        self.body=types.LocateArucoTags.deserialize(self.data)
        self.rscp_data.msg_id = self.msg_id 
        self.rscp_data_pub.publish(self.rscp_data)
        print(f"Body of the Message = {self.body}")     
        
    
    def locate3d_body (self):
        self.body=types.Location3D.deserialize(self.data)
        self.rscp_data.msg_id = self.msg_id
        self.rscp_data.reference = self.body.reference
        self.rscp_data.aruco_coordinates = [self.body.x ,self.body.y,self.body.z]
        self.rscp_data_pub.publish(self.rscp_data)
        print(f"Body of the Message = {self.body}")     

    
    def detection_body(self):
        self.body=types.Detection.deserialize(self.data)
        print(f"Body of the Message = {self.body}")     
    
    def setparameters_body(self):
        self.body=types.SetParameters.deserialize(self.data)
        self.rscp_data.latitude = self.body.parameters["latitude"]
        print(self.rscp_data.latitude)
        self.rscp_data.longitude = self.body.parameters["longitude"]
        self.rscp_data.text = self.body.parameters["text"]
        self.rscp_data.msg_id = self.msg_id
        self.rscp_data_pub.publish(self.rscp_data)
        print(f"Body of the Message = {self.body}")     


    def create_serialize(self,msg_cls: Type[types.MessageBase], *args, **kwargs):
        msg = msg_cls(*args, **kwargs)
        serialized = msg.serialize()
        return(serialized)
    

    def ack_frame(self):
        self.body=self.create_serialize(types.Acknowledge)
        self.frame=Frame.create(0x00,self.body)
        return(self.frame)

    def armdis_frame(self,bool_value):
        self.body=self.create_serialize(types.ArmDisarm, bool_value)
        self.frame=Frame.create(0x01,self.body)
        return(self.frame)
    
    def navigation_to_gps_frame(self, latitude, longitude):
        self.body=self.create_serialize(types.NavigateToGPS, latitude, longitude)
        self.frame=Frame.create(0x02,self.body)
        return(self.frame)
    
    def task_completed_frame(self):
        self.body=self.create_serialize(types.TaskFinished)
        self.frame=Frame.create(0x03,self.body)
        return(self.frame)
    
    def setstage_frame(self, stage):
        self.body = self.create_serialize(types.SetStage,stage)
        self.frame=Frame.create(0x04,self.body)
        return(self.frame)
    
    def text_frame(self,text):
        self.body = self.create_serialize(types.Text,text)
        self.frame=Frame.create(0x05,self.body)
        return(self.frame)
    
    def arucotag_frame(self,tag_id, dictionary):
        self.body = self.create_serialize(types.ArucoTag,tag_id, dictionary)
        self.frame=Frame.create(0x06,self.body)
        return(self.frame)
    
    def locatearucotags_frame(self, entr_tag_id, entr_tag_dict, exit_tag_id, exit_tag_dict):
        self.body = self.create_serialize(types.LocateArucoTags,types.ArucoTag(entr_tag_id, entr_tag_dict),types.ArucoTag(exit_tag_id,exit_tag_dict))
        self.frame=Frame.create(0x07,self.body)
        return(self.frame)
    
    def locate3d_frame(self):
        #not used anywhere
        self.body = self.create_serialize(types.Location3D,1,1,1,"hello world")
        self.frame=Frame.create(0x08,self.body)
        return(self.frame)
    
    def dedect_frame(self,distance, colour):
        self.body=self.create_serialize(types.Detection,distance, colour)
        self.frame=Frame.create(0x09,self.body)
        return(self.frame)

        
    def setparameters_frame(self,dictionary):
        self.body = self.create_serialize(types.SetParameters,dictionary)
        self.frame=Frame.create(0x0A,self.body)
        return(self.frame)
    
    def main(self):
        while(1):
            try:
                data = self.ser.read()  
                if data:
                    #print(f"Received: {data}")
                    self.test_parser(data)
                else:
                    print("No data received within the timeout period.")
            except KeyboardInterrupt:
                print("Exiting...")
        

if __name__ == '__main__':
    RSCP_object = RSCP_Receiver()
    RSCP_object.main()
    rospy.spin()


    


