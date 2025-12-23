from typing import List
from commands2 import Subsystem
from phoenix6 import utils
from Utilities.LLH import LimelightHelpers
from Utilities.LLH import PoseEstimate
from Utilities.LLH import RawFiducial
from phoenix6 import utils
from wpilib import DriverStation
from robotstate import RobotState
from Constants import ConstantValues
from wpilib import SmartDashboard

from subsystems.Drive.driveTrainGenerate import DrivetrainGenerator

class llSystem(Subsystem):
    def __init__(self):
        self.robotState = RobotState.getInstance()
        self.driveTrain = DrivetrainGenerator.getInstance()
        self.Lconstants = ConstantValues.LeftLimelightConstants
        self.Rconstants = ConstantValues.RightLimelightConstants 

        self.max_value = 9999

        self.xyStdDevCoefficient = 0.02
        self.thetaStdDevCoefficient = self.max_value  #0.04

        self.previousLeftEstimate = PoseEstimate()
        self.previousRightEstimate = PoseEstimate()

        self.PE = PoseEstimate()

        self.configfureLimelights()


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

        LimelightHelpers.set_imu_mode(self.Lconstants.CAMERA_NAME, 0)
        LimelightHelpers.set_imu_mode(self.Rconstants.CAMERA_NAME, 0)    
    

    
    def update(self): 
        SmartDashboard.putBoolean("LL left tv",LimelightHelpers.get_tv(self.Lconstants.CAMERA_NAME))
        SmartDashboard.putBoolean("LL right tv",LimelightHelpers.get_tv(self.Rconstants.CAMERA_NAME))        
        """ check if we are moving too fast for an accurate camera measurement """
        shouldAccept = (self.robotState.getChassisSpeedsNorm()<3 
                        and abs(self.robotState.getRotationalSpeedsRPS())<2)
        SmartDashboard.putBoolean("Accept Target",shouldAccept)

        """ set the standard deviations for use in the pose estimator """
        leftStdDev = self.max_value
        leftHeadingStdDev = self.max_value
        rightStdDev = self.max_value
        rightHeadingStdDev = self.max_value

        leftEstimate=None
        rightEstimate=None

        leftEstimate = self.pollLL(self.Lconstants.CAMERA_NAME, self.previousLeftEstimate)
        rightEstimate = self.pollLL(self.Rconstants.CAMERA_NAME, self.previousRightEstimate)
        

        if shouldAccept :
            if leftEstimate is not None:
              if len(leftEstimate.raw_fiducials) > 0:
                closestID,closestTagDist,farthestID,farthestTagDist = self.minmaxDist(leftEstimate.raw_fiducials)
                leftStdDev = self.xyStdDevCoefficient * (leftEstimate.avg_tag_dist ** 2) / leftEstimate.tag_count
                leftHeadingStdDev = self.thetaStdDevCoefficient * (leftEstimate.avg_tag_dist ** 2) / leftEstimate.tag_count
                if leftEstimate.avg_tag_dist > 5:
                    leftStdDev =self.max_value
            
                
                SmartDashboard.putNumber("LL-L closest ID",closestID)    
                SmartDashboard.putNumber("LL-L closest Dist",closestTagDist)    
                SmartDashboard.putNumber("LL-L fartheet ID",farthestID)    
                SmartDashboard.putNumber("LL-L farthest Dist",farthestTagDist)                    
                


            if rightEstimate is not None:
              if len(rightEstimate.raw_fiducials) > 0:
                closestID,closestTagDist,farthestID,farthestTagDist = self.minmaxDist(rightEstimate.raw_fiducials)
                rightStdDev = self.xyStdDevCoefficient * (rightEstimate.avg_tag_dist ** 2) / rightEstimate.tag_count
                rightHeadingStdDev = self.thetaStdDevCoefficient * (rightEstimate.avg_tag_dist ** 2) / rightEstimate.tag_count
                if rightEstimate.avg_tag_dist > 5:
                    rightStdDev =self.max_value
                
                SmartDashboard.putNumber("LL-R closest ID",closestID)    
                SmartDashboard.putNumber("LL-R closest Dist",closestTagDist)    
                SmartDashboard.putNumber("LL-R fartheet ID",farthestID)    
                SmartDashboard.putNumber("LL-R farthest Dist",farthestTagDist)                    

            
            if leftStdDev < rightStdDev:
                SmartDashboard.putNumber("LL Std Dev XY",leftStdDev)                                
                SmartDashboard.putNumber("LL Std Dev Rot",leftHeadingStdDev)                                
                SmartDashboard.putNumber("LL Num Targ",leftEstimate.tag_count)                
                SmartDashboard.putNumber("LL pose X",leftEstimate.pose.translation().X())
                SmartDashboard.putNumber("LL pose Y",leftEstimate.pose.translation().Y()) 
                SmartDashboard.putNumber("LL pose R",leftEstimate.pose.rotation().radians() )                               
                SmartDashboard.putNumber("LL Dist avg",leftEstimate.avg_tag_dist)    
                self.driveTrain.add_vision_measurement(
                        leftEstimate.pose,
                        utils.fpga_to_current_time(leftEstimate.timestamp_seconds),
                        (leftStdDev, leftStdDev, leftHeadingStdDev))
                
            elif rightStdDev < leftStdDev:
                self.driveTrain.add_vision_measurement(
                        rightEstimate.pose,
                        utils.fpga_to_current_time(rightEstimate.timestamp_seconds),
                        (rightStdDev, rightStdDev, rightHeadingStdDev))






    def minmaxDist(self,rf:List[RawFiducial]):
        maxD=0
        maxID=0
        minD=9999
        minID=0
        iMax=len(rf)
        i=0

        while i<iMax:
            if rf[i].dist_to_camera<minD:
                minID=i
                minD=rf[i].dist_to_camera
            if rf[i].dist_to_camera>maxD:
                maxID=i
                maxD=rf[i].dist_to_camera                
            i=i+1
        return rf[minID].id,minD,rf[maxID].id,maxD




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
                


    def pollLLMT1(self,id,previousEstimate: PoseEstimate):

        if (LimelightHelpers.get_tv(id)):
            if previousEstimate is not None:
                oldTimestamp =  previousEstimate.timestamp_seconds 
            else:
                oldTimestamp = self.max_value
                
            newEstimate = LimelightHelpers.get_botpose_estimate_wpiblue(id)
            
            if newEstimate is not None:
                
                if newEstimate.timestamp_seconds == oldTimestamp:
                    newEstimate = None
                else:
                    previousEstimate = newEstimate
        else:
            newEstimate = None 
                       
        return newEstimate            
        

        
    def periodic(self):
        self.update()
                        