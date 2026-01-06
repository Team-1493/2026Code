from wpilib import CAN,CANData, RobotBase

import robot
from subsystems.Vision import limelight_system


class LaserCAN:
    instance = None
    @staticmethod
    def getInstance():
        if LaserCAN.instance == None:
            LaserCAN.instance = LaserCAN(0,6,6)
            print("**********************  LaserCAN  **********************") 
        return LaserCAN.instance
    
    
    def __init__(self, can_id: int,  manufacturer: int, devicetType: int):
        # Create CAN device
        self.can = CAN(can_id, manufacturer,devicetType)
        print("Created LC")
        self.packet=CANData()
        self.status=1
        self.dist=-1

        
    def get_distance_mm(self,api:int) -> int | None:
        # Read CAN frame

        if RobotBase.isSimulation():
            b = limelight_system.LLsystem.readPacketNew(api,self.packet)
        else:
            b = self.can.readPacketNew(api, self.packet)

        if b is False:
            self.status=1
            return None
        #print("packet  ",self.packet.data[0],"  ",self.packet.data[1],"  ",self.packet.data[2],"   ",self.packet.data[3],"   ",self.packet.data[4],"  ",self.packet.data[5],"  ",self.packet.data[6],"   ",self.packet.data[7])

        distance_mm = distance_mm = self.packet.data[2] | (self.packet.data[1] << 8)
##        ambient = self.packet.data[5] | (self.packet.data[6] << 8)
 #       strength = self.packet.data[4 ]| (self.packet.data[3] << 8)
        self.status = self.packet.data[0]
 #       print("d = ",distance_mm,"  amb = ",ambient,"   str = ",strength,"   status = ",status)                        
        return distance_mm


    def get_status(self):
        return self.status

    def get_distance_meters(self) -> float | None:
        dist_mm = self.get_distance_mm(0)

        if dist_mm is None:
            return -1
        else: 
            dist_m = dist_mm/1000.0
            self.dist = dist_m
            return dist_m
