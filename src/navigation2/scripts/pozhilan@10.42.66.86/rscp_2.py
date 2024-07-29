from rscp.frame_parser import FrameParser
from rscp.message import types
from typing import List, Type
from rscp_1 import Sender
import serial

 

class receiver():
    def __init__(self,ser) -> None:
        self.data=None
        self.rx=bytearray()
        self.sender=Sender()
        self.ser=ser

    # def create_deserialize(self,msg_cls: Type[types.MessageBase], *args, **kwargs):
    #     msg = msg_cls(*args, **kwargs)
    #     deserialized = msg.deserialize(*args, **kwargs)
    #     print(deserialized)

    def on_update(self,frame):
        self.data=frame.data
        if frame.msg_id == 0:
            self.acknowledge_body()
            self.rx = bytearray()

        elif frame.msg_id == 1:
            self.armdisarm_body()
            self.rx=bytearray()

        elif frame.msg_id == 2:
            self.navigatetoGPS_body()
            self.rx=bytearray()


        elif frame.msg_id == 3:
            self.taskfinished_body()
            self.rx=bytearray()

        elif frame.msg_id == 4:
            self.setstage_body()
            self.rx=bytearray()
            
        elif frame.msg_id == 5:
            self.text()
            self.rx=bytearray()

        elif frame.msg_id == 6:
            self.arucotag_body()
            self.rx=bytearray()

        elif frame.msg_id == 7:
            self.locatearucotags_body()
            self.rx=bytearray()

        elif frame.msg_id == 8:
            self.locate3d_body()
            self.rx=bytearray()

        elif frame.msg_id == 9:
            self.detection_body()
            self.rx=bytearray()

        elif frame.msg_id == 10:
            self.setparameters_body()
            self.rx=bytearray()

        


    

    def test_parser(self,bytestream):
        # frame: bytes =bytestream
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
        print(f"Body of the Message = {self.body}")     
        self.rscp_data.latitude = self.body.parameters["latitude"]
        self.rscp_data.longitude = self.body.parameters["longitude"]
        self.rscp_data_pub.publish(self.rscp_data)
        return self.body

        


if __name__=='__main__':
    hi=receiver()
    hi.test_parser()
    # body=hi.armdisarm_body()
    # print(body.arm)
    # print(body.distance)
    body=bytearray()
    

