from rscp.frame_parser import Frame
from rscp.message import types
from typing import List, Type

class Sender:
    def __init__(self) -> None:
        self.rx = bytearray()

    def create_serialize(self,msg_cls: Type[types.MessageBase], *args, **kwargs):
        msg = msg_cls(*args, **kwargs)
        serialized = msg.serialize()
        return(serialized)
    
    # def crt_ack(self):
    #     self.body=self.create_serialze(types.Acknowledge)

    # def crt_armdis(self):
    #     self.body=self.create_serialze(types.ArmDisarm, True)

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

    def taskfinshed_frame(self):
        self.body=self.create_serialize(types.TaskFinished)
        self.frame=Frame.create(0x03,self.body)
        return(self.frame)
        
    def setparameters_frame(self,dictionary):
        self.body = self.create_serialize(types.SetParameters,dictionary)
        self.frame=Frame.create(0x0A,self.body)
        return(self.frame)


    

    

if __name__=="__main__":
    hi=Sender()
    hi.ack_frame()
    hi.armdis_frame() 
    # hi.dedect_frame()
    # hi.test_navigation_to_gps_frame() 
    # hi.taskfinshed_frame()  


