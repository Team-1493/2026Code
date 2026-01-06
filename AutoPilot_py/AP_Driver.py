# all distances in meters, all angles in radians

from AutoPilot_py.AP import Autopilot
from AutoPilot_py.Contraints import APConstraints
from AutoPilot_py.Profile import APProfile

class ap_driver:
    instance=None
    
    @staticmethod
    def getInstance():
        if ap_driver.instance == None:
            ap_driver.instance = ap_driver()
            print("**********************  AP Driver Created  **********************") 
        return ap_driver.instance


    def __init__(self):        
        self.kConstraints = APConstraints().with_acceleration(5.0).with_jerk(2.0)
        self.kProfile = (
            APProfile(self.kConstraints).
            with_ErrorXY(0.03).
            with_ErrorTheta(0.017).
            with_BeelineRadius(0.08)
        )
        self.kAutopilot = Autopilot(self.kProfile)