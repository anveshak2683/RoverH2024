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
            self.rx=bytearray()
            self.frame=self.sender.ack_frame()
            self.ser.write(self.frame)

        elif frame.msg_id == 1:
            self.armdisarm_body()
            self.rx=bytearray()
            self.frame=self.sender.ack_frame()
            self.ser.write(self.frame)

        elif frame.msg_id == 2:
            self.navigatetoGPS_body()
            self.rx=bytearray()
            self.frame=self.sender.ack_frame()
            self.ser.write(self.frame)


        elif frame.msg_id == 3:
            self.taskfinished_body()
            self.rx=bytearray()
            self.frame=self.sender.ack_frame()
            self.ser.write(self.frame)

        elif frame.msg_id == 4:
            self.setstage_body()
            self.rx=bytearray()
            self.frame=self.sender.ack_frame()
            self.ser.write(self.frame)
            
        elif frame.msg_id == 5:
            self.text()
            self.rx=bytearray()
            self.frame=self.sender.ack_frame()
            self.ser.write(self.frame)

        elif frame.msg_id == 6:
            self.arucotag_body()
            self.rx=bytearray()
            self.frame=self.sender.ack_frame()
            self.ser.write(self.frame)

        elif frame.msg_id == 7:
            self.locatearucotags_body()
            self.rx=bytearray()
            self.frame=self.sender.ack_frame()
            self.ser.write(self.frame)

        elif frame.msg_id == 8:
            self.locate3d_body()
            self.rx=bytearray()
            self.frame=self.sender.ack_frame()
            self.ser.write(self.frame)

        elif frame.msg_id == 9:
            self.detection_body()
            self.rx=bytearray()
            self.frame=self.sender.ack_frame()
            self.ser.write(self.frame)

        elif frame.msg_id == 10:
            self.setparameters_body()
            self.rx=bytearray()
            self.frame=self.sender.ack_frame()
            self.ser.write(self.frame)

        


    

    def test_parser(self,bytestream):
        # frame: bytes =bytestream
        self.rx.extend(bytestream)

        frame_parser = FrameParser(self.on_update)

        for byte in self.rx:
            frame_parser.process(byte)

    def acknowledge_body(self):
        self.body = types.Acknowledge.deserialize(self.data)

        return (self.body)
    
    def armdisarm_body(self):
        self.body=types.ArmDisarm.deserialize(self.data)
        
        
        print(self.body.arm)
    
    def navigatetoGPS_body(self):
        self.body = types.NavigateToGPS.deserialize(self.data)
        print (self.body)
    
    
    def taskfinished_body(self):
        self.body = types.TaskFinished.deserialize(self.data)
        print(self.body)
    
    def setstage_body(self):
        self.body = types.SetStage.deserialize(self.data)
        print(self.body)
    
    def text(self):
        self.body = types.Text.deserialize(self.data)
        print(self.body)
    
    
    def arucotag_body(self):
        self.body=types.ArucoTag.deserialize(self.data)
        print (self.body)
    
    def locatearucotags_body(self):
        self.body=types.LocateArucoTags.deserialize(self.data)
        print(self.body)
    
    def locate3d_body (self):
        self.body=types.Location3D.deserialize(self.data)
        print(self.body)
    
    def detection_body(self):
        self.body=types.Detection.deserialize(self.data)
        print(self.body)
    
    def setparameters_body(self):
        self.body=types.SetParameters.deserialize(self.data)
        return self.body

        


if __name__=='__main__':
    hi=receiver()
    hi.test_parser()
    # body=hi.armdisarm_body()
    # print(body.arm)
    # print(body.distance)
    body=bytearray()
    

