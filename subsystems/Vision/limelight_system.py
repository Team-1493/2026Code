from typing import List
from commands2 import Subsystem
from phoenix6 import utils
from Utilities.LLH import LimelightHelpers
from Utilities.LLH import PoseEstimate
from Utilities.LLH import RawFiducial
from phoenix6 import utils
from wpilib import CANData, DriverStation, RobotBase
from robot_state import RobotState
from  Constants1 import ConstantValues
from wpilib import SmartDashboard

from subsystems.Drive.drivetrain_generator import DrivetrainGenerator
from subsystems.laser_can import LaserCAN

class LLsystem(Subsystem):
    LC_dist = 32000
    LC_cam = None
    instance = None

    @staticmethod
    def getInstance():
        if LLsystem.instance == None:
            LLsystem.instance = LLsystem()
            print("********************** LL System  **********************") 
        return LLsystem.instance
    

    def __init__(self):
        self.closestTagDist=9999
        self.robotState = RobotState.getInstance()
        self.driveTrain = DrivetrainGenerator.getInstance()
        self.constants =  ConstantValues.LimelightConstants
        LLsystem.LC_cam = self.constants.CAMERA_NAME_L

        self.max_value = 9999

        self.previousLeftEstimate = PoseEstimate()
        self.previousRightEstimate = PoseEstimate()


        self.PE = PoseEstimate()

        self.configfureLimelights()
        self.zeroAndseedIMU(0)

        self.LC = LaserCAN.getInstance()

    def periodic(self):
        self.update()


    def update(self): 
        self.closestTagDist = None
        SmartDashboard.putBoolean("LL left tv",LimelightHelpers.get_tv(self.constants.CAMERA_NAME_L))
        SmartDashboard.putBoolean("LL right tv",LimelightHelpers.get_tv(self.constants.CAMERA_NAME_R))        
        """ check if we are moving too fast for an accurate camera measurement """
        shouldAccept = (self.robotState.getChassisSpeedsNorm()<3 
                        and abs(self.robotState.getRotationalSpeedsRPS())<2)
        SmartDashboard.putBoolean("Accept Target",shouldAccept)

        """ set the standard deviations for use in the pose estimator """
        leftEstimate_avgDist = self.max_value
        rightEstimate_avgDist = self.max_value
        estimate = None
        
        leftEstimate = self.pollLL(self.constants.CAMERA_NAME_L, self.previousLeftEstimate)
        rightEstimate = self.pollLL(self.constants.CAMERA_NAME_R, self.previousRightEstimate)
        if leftEstimate is not None: leftEstimate_avgDist = leftEstimate.avg_tag_dist
        if rightEstimate is not None: rightEstimate_avgDist = rightEstimate.avg_tag_dist        
  
        if leftEstimate_avgDist<rightEstimate_avgDist:
            estimate = leftEstimate
            cam="left"
        elif leftEstimate_avgDist>rightEstimate_avgDist:
            estimate = rightEstimate
            cam="right"
        else:
            cam = "none"
        SmartDashboard.putString("Cam",cam)

        if shouldAccept :
            if estimate is not None:
              if len(estimate.raw_fiducials) > 0:
                closestID,self.closestTagDist = self.minDist(estimate.raw_fiducials)
                stdDev = self.constants.STD_DEV_COEFF_XY * (self.closestTagDist ** 2) / estimate.tag_count
#                stdDev = 0.02 + 0.05 * self.closestTagDist
                headingStdDev = self.constants.STD_DEV_COEFF_THETA * (self.closestTagDist** 2) / estimate.tag_count
                if estimate.avg_tag_dist > self.constants.CAMERA_CUTOFF_DISTANCE:
                    stdDev = self.max_value
                        
                LLsystem.LC_dist = self.closestTagDist
                SmartDashboard.putNumber("LL closest ID",closestID)    
                SmartDashboard.putNumber("LL closest Dist",round(self.closestTagDist,3))                                    
                SmartDashboard.putNumber("LL Num Targ",estimate.tag_count)                
                SmartDashboard.putNumber("LL pose X",round(estimate.pose.translation().X(),3))
                SmartDashboard.putNumber("LL pose Y",round(estimate.pose.translation().Y(),3)) 
                SmartDashboard.putNumber("LL pose R",round(estimate.pose.rotation().radians(),3) )                               
                SmartDashboard.putNumber("LL Dist avg",round(estimate.avg_tag_dist,3))                
                SmartDashboard.putNumber("LL Area avg",round(estimate.avg_tag_area,3))    
                SmartDashboard.putNumber("LL Pitch_ty",LimelightHelpers.get_ty(self.constants.CAMERA_NAME_L))                                 
                SmartDashboard.putNumber("LL Pitch_tync",LimelightHelpers.get_tync(self.constants.CAMERA_NAME_L))
                SmartDashboard.putNumber("LL Yaw_tx",LimelightHelpers.get_tx(self.constants.CAMERA_NAME_L))                                 
                SmartDashboard.putNumber("LL Yaw_txnc",LimelightHelpers.get_txnc(self.constants.CAMERA_NAME_L))                
                SmartDashboard.putNumber("LL Std Dev XY",round(stdDev,3))  
                SmartDashboard.putNumber("LL Std Dev Theta",round(headingStdDev,3))  
                
                self.driveTrain.add_vision_measurement(
                        estimate.pose,
                        utils.fpga_to_current_time(estimate.timestamp_seconds),
                        (stdDev, stdDev, headingStdDev))
                

#    def get_minDist():
#        return         


    def minDist(self,rf:List[RawFiducial]):
        minD=9999
        minID=0
        iMax=len(rf)
        i=0

        while i<iMax:
            if rf[i].dist_to_camera<minD:
                minID=i
                minD=rf[i].dist_to_camera         
            i=i+1
        return rf[minID].id,minD




    def pollLL(self,id,previousEstimate: PoseEstimate): 
        LimelightHelpers.set_robot_orientation(
                id, self.robotState.getRotationDeg(), 0, 0, 0, 0, 0)
        if (LimelightHelpers.get_tv(id)):
            if previousEstimate is not None:
                oldTimestamp =  previousEstimate.timestamp_seconds 
            else:
                oldTimestamp = self.max_value
                
            newEstimate = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(id)
            
            
            if newEstimate is not None:
                
                if newEstimate.timestamp_seconds == oldTimestamp:
                    newEstimate = None
                else:
                    previousEstimate = newEstimate
        else:
            newEstimate = None 

        return newEstimate         
        

    def configfureLimelights(self):

        LimelightHelpers.set_camerapose_robotspace(
                self.constants.CAMERA_NAME_L,
                self.constants.X_OFFSET_L, 
                self.constants.Y_OFFSET_L,
                self.constants.Z_OFFSET_L,
                self.constants.THETA_X_OFFSET_L,
                self.constants.THETA_Y_OFFSET_L,
                self.constants.THETA_Z_OFFSET_L)
        
        LimelightHelpers.set_camerapose_robotspace(
                self.constants.CAMERA_NAME_R,
                self.constants.X_OFFSET_R, 
                self.constants.Y_OFFSET_R,
                self.constants.Z_OFFSET_R,
                self.constants.THETA_X_OFFSET_R,
                self.constants.THETA_Y_OFFSET_R,
                self.constants.THETA_Z_OFFSET_R)   

    
    def zeroAndseedIMU(self,rot=None):
        if rot is None:
            rot=self.robotState.getRotationRad()

        LimelightHelpers.set_robot_orientation(
                self.constants.CAMERA_NAME_L, rot, 0, 0, 0, 0, 0)
        LimelightHelpers.set_robot_orientation(
                self.constants.CAMERA_NAME_R, rot, 0, 0, 0, 0, 0)
        
        # use external IMU, seed internal IMU with value from set_robot_orientation
        LimelightHelpers.set_imu_mode(self.constants.CAMERA_NAME_L, 1)
        LimelightHelpers.set_imu_mode(self.constants.CAMERA_NAME_R, 1)    


    def set_IMU_Mode(self, mode:int):
        LimelightHelpers.set_imu_mode(self.constants.CAMERA_NAME_L, mode)
        LimelightHelpers.set_imu_mode(self.constants.CAMERA_NAME_R, mode)    

# only these tags can be detected, limits what tags used for megatag
    def set_id_filter_override(self,idList:List[int]):
        LimelightHelpers.set_fiducial_id_filters_override(self.constants.CAMERA_NAME_L,idList)
        LimelightHelpers.set_fiducial_id_filters_override(self.constants.CAMERA_NAME_R,idList)        

# choose preferred tag for best target, determines which tag used for tx, ty, ta
# megatag still uses all visible tags
    def set_priority_tag(self,id):
        LimelightHelpers.set_priority_tag_id(self.constants.CAMERA_NAME_L,2)        
        LimelightHelpers.set_priority_tag_id(self.constants.CAMERA_NAME_R,2)        



# Use this to for simulating the lasercan.
    def readPacketNew(api: int, packet:CANData):
        status = 1
        dist = int(32000)
        b = False

        if LimelightHelpers.get_tv(LLsystem.LC_cam): 
                if LLsystem.LC_dist is not None:
                    if LLsystem.LC_dist<4:
                        status=0
                        dist=int(LLsystem.LC_dist*1000)
                        b = True

        high_byte = dist >> 8
        low_byte = dist & 0xFF
        
        packet.data[0] = status
        packet.data[2] = low_byte
        packet.data[1] = high_byte    

        return b            

                        