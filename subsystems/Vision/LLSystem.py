from typing import List
from commands2 import Subsystem
from phoenix6 import utils
from Utilities.LLH import LimelightHelpers
from Utilities.LLH import PoseEstimate
from Utilities.LLH import RawFiducial
from phoenix6 import utils
from wpilib import CANData, DriverStation, RobotBase
from robotstate import RobotState
from Constants import ConstantValues
from wpilib import SmartDashboard

from subsystems.Drive.driveTrainGenerate import DrivetrainGenerator

class llSystem(Subsystem):
    LC_dist = 32000
    LC_cam = ConstantValues.LeftLimelightConstants.CAMERA_NAME

    def __init__(self):
        
        self.closestTagDist=9999
        self.robotState = RobotState.getInstance()
        self.driveTrain = DrivetrainGenerator.getInstance()
        self.Lconstants = ConstantValues.LeftLimelightConstants
        self.Rconstants = ConstantValues.RightLimelightConstants 

        self.max_value = 9999

        self.xyStdDevCoefficient = 0.02
        self.thetaStdDevCoefficient = .1 #0.04 or self,max_value

        self.previousLeftEstimate = PoseEstimate()
        self.previousRightEstimate = PoseEstimate()


        self.PE = PoseEstimate()

        self.configfureLimelights()
        self.zeroAndseedIMU(0)

    def periodic(self):
        self.update()


    def update(self): 
        self.closestTagDist = None
        SmartDashboard.putBoolean("LL left tv",LimelightHelpers.get_tv(self.Lconstants.CAMERA_NAME))
        SmartDashboard.putBoolean("LL right tv",LimelightHelpers.get_tv(self.Rconstants.CAMERA_NAME))        
        """ check if we are moving too fast for an accurate camera measurement """
        shouldAccept = (self.robotState.getChassisSpeedsNorm()<3 
                        and abs(self.robotState.getRotationalSpeedsRPS())<2)
        SmartDashboard.putBoolean("Accept Target",shouldAccept)

        """ set the standard deviations for use in the pose estimator """
        leftEstimate_avgDist = self.max_value
        rightEstimate_avgDist = self.max_value
        estimate = None

        leftEstimate = self.pollLL(self.Lconstants.CAMERA_NAME, self.previousLeftEstimate)
        rightEstimate = self.pollLL(self.Rconstants.CAMERA_NAME, self.previousRightEstimate)
        
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
                stdDev = self.xyStdDevCoefficient * (self.closestTagDist ** 2) / estimate.tag_count
                headingStdDev = self.thetaStdDevCoefficient * (self.closestTagDist** 2) / estimate.tag_count
                if estimate.avg_tag_dist > 5:
                    stdDev = self.max_value
                        
                llSystem.LC_dist = self.closestTagDist
                SmartDashboard.putNumber("LL closest ID",closestID)    
                SmartDashboard.putNumber("LL closest Dist",self.closestTagDist)                                    
                SmartDashboard.putNumber("LL Std Dev XY",stdDev)                                
                SmartDashboard.putNumber("LL Std Dev Rot",headingStdDev)                                
                SmartDashboard.putNumber("LL Num Targ",estimate.tag_count)                
                SmartDashboard.putNumber("LL pose X",estimate.pose.translation().X())
                SmartDashboard.putNumber("LL pose Y",estimate.pose.translation().Y()) 
                SmartDashboard.putNumber("LL pose R",estimate.pose.rotation().radians() )                               
                SmartDashboard.putNumber("LL Dist avg",estimate.avg_tag_dist)
                SmartDashboard.putNumber("LL Area avg",estimate.avg_tag_area)    
                SmartDashboard.putNumber("LL Pitch_ty",LimelightHelpers.get_ty(self.Lconstants.CAMERA_NAME))                                 
                SmartDashboard.putNumber("LL Pitch_tync",LimelightHelpers.get_tync(self.Lconstants.CAMERA_NAME))
                SmartDashboard.putNumber("LL Yaw_tx",LimelightHelpers.get_tx(self.Lconstants.CAMERA_NAME))                                 
                SmartDashboard.putNumber("LL Yaw_txnc",LimelightHelpers.get_txnc(self.Lconstants.CAMERA_NAME))                


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
                self.Lconstants.CAMERA_NAME,
                self.Lconstants.X_OFFSET, 
                self.Lconstants.Y_OFFSET,
                self.Lconstants.Z_OFFSET,
                self.Lconstants.THETA_X_OFFSET,
                self.Lconstants.THETA_Y_OFFSET,
                self.Lconstants.THETA_Z_OFFSET)
        
        LimelightHelpers.set_camerapose_robotspace(
                self.Rconstants.CAMERA_NAME,
                self.Rconstants.X_OFFSET, 
                self.Rconstants.Y_OFFSET,
                self.Rconstants.Z_OFFSET,
                self.Rconstants.THETA_X_OFFSET,
                self.Rconstants.THETA_Y_OFFSET,
                self.Rconstants.THETA_Z_OFFSET)   

    
    def zeroAndseedIMU(self,rot=None):
        if rot is None:
            rot=self.robotState.getRotationRad()

        LimelightHelpers.set_robot_orientation(
                self.Lconstants.CAMERA_NAME, rot, 0, 0, 0, 0, 0)
        LimelightHelpers.set_robot_orientation(
                self.Rconstants.CAMERA_NAME, rot, 0, 0, 0, 0, 0)
        
        # use external IMU, seed internal IMU with value from set_robot_orientation
        LimelightHelpers.set_imu_mode(self.Lconstants.CAMERA_NAME, 1)
        LimelightHelpers.set_imu_mode(self.Rconstants.CAMERA_NAME, 1)    


    def set_IMU_Mode(self, mode:int):
        LimelightHelpers.set_imu_mode(self.Lconstants.CAMERA_NAME, mode)
        LimelightHelpers.set_imu_mode(self.Rconstants.CAMERA_NAME, mode)    

# only these tags can be detected, limits what tags used for megatag
    def set_id_filter_override(self,idList:List[int]):
        LimelightHelpers.set_fiducial_id_filters_override(self.Lconstants.CAMERA_NAME,idList)
        LimelightHelpers.set_fiducial_id_filters_override(self.Rconstants.CAMERA_NAME,idList)        

# choose preferred tag for best target, determines which tag used for tx, ty, ta
# megatag still uses all visible tags
    def set_priority_tag(self,id):
        LimelightHelpers.set_priority_tag_id(self.Lconstants.CAMERA_NAME,2)        
        LimelightHelpers.set_priority_tag_id(self.Rconstants.CAMERA_NAME,2)        



# Use this to for simulating the lasercan.
    def readPacketNew(api: int, packet:CANData):
        status = 1
        dist = int(32000)
        b = False

        if LimelightHelpers.get_tv(llSystem.LC_cam): 
                if llSystem.LC_dist is not None:
                    if llSystem.LC_dist<4:
                        status=0
                        dist=int(llSystem.LC_dist*1000)
                        b = True

        high_byte = dist >> 8
        low_byte = dist & 0xFF
        
        packet.data[0] = status
        packet.data[2] = low_byte
        packet.data[1] = high_byte    

        return b            

                        