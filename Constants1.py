from math import pi
from robotpy_apriltag import  AprilTag,AprilTagField,AprilTagFieldLayout
from wpilib import SmartDashboard
from wpimath.geometry import Pose3d, Rotation3d,Translation3d

from generated.tuner_constants import TunerConstants

class ConstantValues():
    instance=None
    
    @staticmethod
    def getInstance():
        if ConstantValues.instance == None:
            ConstantValues.instance = ConstantValues()
            print("**********************  ConstantValues  **********************") 
            ConstantValues.write_constants_dashboard()
        return ConstantValues.instance

    class DriveConstants():
        
        TELEOP_kP = TunerConstants._drive_gains.k_p
        TELEOP_kS = TunerConstants._drive_gains.k_s
        TELEOP_kV =  TunerConstants._drive_gains.k_v
        TELEOP_kA =  TunerConstants._drive_gains.k_a

        AUTO_kP = 0.1
        AUTO_kS = 0
        AUTO_kV = 0.124
        AUTO_kA = 0        

    class LimelightConstants():
        CAMERA_NAME_R = "limelightRight"
        X_OFFSET_R = 0 # forward positive
        Y_OFFSET_R = 0 # right positive
        Z_OFFSET_R = 0 # up positive
        THETA_X_OFFSET_R = 0 # roll
        THETA_Y_OFFSET_R = 0 # pitch
        THETA_Z_OFFSET_R = 0 # yaw

        CAMERA_NAME_L = "limelightLeft"
        X_OFFSET_L = 0 # forward positive
        Y_OFFSET_L = 0 # right positive
        Z_OFFSET_L = 0 # up positive
        THETA_X_OFFSET_L = 0 # roll
        THETA_Y_OFFSET_L = 0 # pitch
        THETA_Z_OFFSET_L = 0 # yaw

        STD_DEV_COEFF_XY = 0.1 #0.05
        STD_DEV_COEFF_THETA = 999 #0.04 or self,max_value

        CAMERA_CUTOFF_DISTANCE = 3 # meters, above this distance std's set to max        


    class AutoBuilderConstants():
        AUTOBUILDER_XY_kP = 4        
        AUTOBUILDER_XY_kD = 0
        AUTOBUILDER_THETA_kP = 4        
        AUTOBUILDER_THETA_kD = 0        

    class HeadingControllerConstants():
        
        HEADINGCONTROLLER_KP = 16.0
        HEADINGCONTROLLER_KD = 0.0
        HEADINGCONTROLLER_VMAX = 2.0
        HEADINGCONTROLLER_AMAX = 4.0 
        HEADINGCONTROLLER_TOLERANCE = 0.017
        HEADINGCONTROLLER_RATE_TOLERANCE = 0.03                 
    
    class DriveToGoalCameraConstants():
        kLateralTolerance = .5
        kForwardTolerance = 0.03 # meters
        kRotTolerance = 0.017
        kEndTriggerDebounce = 0.06 #seconds
        kTimeoutTeleop = 1.5 #seconds
        kTimeoutAuto = 0.6 #seconds
        kForVmax = 1
        kForAmax = 6
        kForP = 8                
        kForD = .1
        kLatVmax = .5
        kLatAmax = 1
        kLatP = 0.2               
        kLatD = 0.0
        kRotVmax = 2
        kRotAmax = 4
        kRotP = 12                
        kRotD = 1    


    class VisionConstants():    
        tag1=AprilTag()
        tag1.ID = 1
        tag1.pose = Pose3d(Translation3d(16.697, 0.655, 0.308), Rotation3d(0.000, 0.000, 2.199))

        tag2=AprilTag()
        tag2.ID = 2
        tag2.pose = Pose3d(Translation3d(16.697, 7.396, 0.308),Rotation3d(0.000, 0.000, 4.084))

        tag3=AprilTag()
        tag3.ID = 3
        tag3.pose = Pose3d(Translation3d(11.561, 8.056, 0.308), Rotation3d(0.000, 0.000, 4.712))

        tag4=AprilTag()
        tag4.ID = 4
        tag4.pose = Pose3d(Translation3d(9.276, 6.138,	0.3088), Rotation3d(0.000,	0.524, 0.000))

        tag5=AprilTag()
        tag5.ID = 5
        tag5.pose = Pose3d(Translation3d(9.276, 1.915,	0.308), Rotation3d(0.000,	0.524, 0.000))

        tag6=AprilTag()
        tag6.ID = 6
        tag6.pose = Pose3d(Translation3d(13.474, 3.306, 0.308), Rotation3d(0.000, 0, 5.236))

        tag7=AprilTag()
        tag7.ID = 7
        tag7.pose = Pose3d(Translation3d(13.891, 4.026, 0.308), Rotation3d(0.000, 0.000, 0.000))

        tag8=AprilTag()
        tag8.ID = 8
        tag8.pose = Pose3d(Translation3d(13.474, 4.745, 0.308), Rotation3d(0.000, 0.000, 1.047))

        tag9=AprilTag()
        tag9.ID = 9
        tag9.pose = Pose3d(Translation3d(12.643, 4.745, 0.308), Rotation3d(0.000, 0.000, 2.094))

        tag10=AprilTag()
        tag10.ID = 10
        tag10.pose = Pose3d(Translation3d(12.227, 4.026, 0.308), Rotation3d(0.000, 0.000, 3.142))

        tag11=AprilTag()
        tag11.ID = 11
        tag11.pose = Pose3d(Translation3d(12.643, 3.306, 0.308), Rotation3d(0.000, 0.000, 4.189))

        tag12=AprilTag()
        tag12.ID = 12
        tag12.pose = Pose3d(Translation3d(0.851, 0.655, 0.308),  Rotation3d(0.000, 0.000, 0.942))

        tag13=AprilTag()
        tag13.ID = 13
        tag13.pose = Pose3d(Translation3d(0.851, 7.396, 0.308), Rotation3d(0.000, 0, 5.341))

        tag14=AprilTag()
        tag14.ID = 14
        tag14.pose = Pose3d(Translation3d(8.272, 6.138, 0.308),  Rotation3d(0.000, 0.524, 3.142))

        tag15=AprilTag()
        tag15.ID = 15
        tag15.pose = Pose3d(Translation3d(8.272, 1.915, 0.308),  Rotation3d(0.000, 0.524, 3.142))

        tag16=AprilTag()
        tag16.ID = 16
        tag16.pose = Pose3d(Translation3d(5.988, -0.004,	0.308), Rotation3d(0.000,	0.000, 1.571))

        tag17=AprilTag()
        tag17.ID = 17
        tag17.pose = Pose3d(Translation3d(4.074, 3.306,  0.308),  Rotation3d(0.000, 0.000, 4.189))
        
        tag18=AprilTag()                
        tag18.ID = 18
        tag18.pose = Pose3d(Translation3d(3.658, 4.026, 0.308),  Rotation3d(0.000, 0.000, 3.142))
        
        tag19=AprilTag()
        tag19.ID = 19
        tag19.pose = Pose3d(Translation3d(4.074, 4.745, 0.308),  Rotation3d(0.000, 0.000, 2.094))

        tag20=AprilTag()
        tag20.ID = 20
        tag20.pose = Pose3d(Translation3d( 4.905, 4.745, 0.308),  Rotation3d(0.000, 0.000, 1.047))

        tag21=AprilTag()        
        tag21.ID = 21  
        tag21.pose = Pose3d(Translation3d(5.321, 4.026, 0.308),  Rotation3d(0.000, 0.000, 0.000))
        
        # tags facing robot have r3 = 180 degrees
        
        tags_list = [tag1,tag2,tag3,tag4,tag5,tag6,tag7,tag8,tag9,tag10,tag11,
                    tag12,tag13,tag14,tag15,tag16,tag17,tag18,tag19,tag20,tag21]

        field_length_meters = 16.48 # Example field dimensions
        field_width_meters = 8.11 # Example field dimensions
        field_layout = AprilTagFieldLayout(tags_list, field_length_meters, field_width_meters)

        
    
    @staticmethod
    def update_constants():

       ### Update Constants for Drive
        ConstantValues.DriveConstants.TELEOP_kP = SmartDashboard.getNumber("Drive Teleop kP",ConstantValues.DriveConstants.TELEOP_kP)
        ConstantValues.DriveConstants.TELEOP_kS = SmartDashboard.getNumber("Drive Teleop kS",ConstantValues.DriveConstants.TELEOP_kS) 
        ConstantValues.DriveConstants.TELEOP_kV = SmartDashboard.getNumber("Drive Teleop kV",ConstantValues.DriveConstants.TELEOP_kV)
        ConstantValues.DriveConstants.TELEOP_kA = SmartDashboard.getNumber("Drive Teleop kA",ConstantValues.DriveConstants.TELEOP_kA)                           
        ConstantValues.DriveConstants.AUTO_kP = SmartDashboard.getNumber("Drive Auto kP",ConstantValues.DriveConstants.AUTO_kP)
        ConstantValues.DriveConstants.AUTO_kS = SmartDashboard.getNumber("Drive Auto kS",ConstantValues.DriveConstants.AUTO_kS)
        ConstantValues.DriveConstants.AUTO_kV = SmartDashboard.getNumber("Drive Auto kV",ConstantValues.DriveConstants.AUTO_kV)
        ConstantValues.DriveConstants.AUTO_kA = SmartDashboard.getNumber("Drive Auto kA",ConstantValues.DriveConstants.AUTO_kA)                         

        ### Update Constants for DriveToGoalCamera
        ConstantValues.DriveToGoalCameraConstants.kLateralTolerance = SmartDashboard.getNumber("DTG_Cam LatTol",ConstantValues.DriveToGoalCameraConstants.kLateralTolerance)      
        ConstantValues.DriveToGoalCameraConstants.kForwardTolerance = SmartDashboard.getNumber("DTG_Cam ForTol",ConstantValues.DriveToGoalCameraConstants.kForwardTolerance)
        ConstantValues.DriveToGoalCameraConstants.kRotTolerance = SmartDashboard.getNumber("DTG_Cam RotTol",ConstantValues.DriveToGoalCameraConstants.kRotTolerance)
        ConstantValues.DriveToGoalCameraConstants.kEndTriggerDebounce = SmartDashboard.getNumber("DTG_Cam Debounce",ConstantValues.DriveToGoalCameraConstants.kEndTriggerDebounce)
        ConstantValues.DriveToGoalCameraConstants.kTimeoutTeleop = SmartDashboard.getNumber("DTG_Cam Timeout Tele",ConstantValues.DriveToGoalCameraConstants.kTimeoutTeleop)
        ConstantValues.DriveToGoalCameraConstants.kTimeoutAuto = SmartDashboard.getNumber("DTG_Cam TimeoutAuto",ConstantValues.DriveToGoalCameraConstants.kTimeoutAuto)    
        ConstantValues.DriveToGoalCameraConstants.kForP = SmartDashboard.getNumber("DTG_Cam For kP",ConstantValues.DriveToGoalCameraConstants.kForP)
        ConstantValues.DriveToGoalCameraConstants.kForD = SmartDashboard.getNumber("DTG_Cam For kD",ConstantValues.DriveToGoalCameraConstants.kForD)
        ConstantValues.DriveToGoalCameraConstants.kForVmax = SmartDashboard.getNumber("DTG_Cam For Vmax",ConstantValues.DriveToGoalCameraConstants.kForVmax) 
        ConstantValues.DriveToGoalCameraConstants.kForAmax = SmartDashboard.getNumber("DTG_Cam For Amax",ConstantValues.DriveToGoalCameraConstants.kForAmax)                 
        ConstantValues.DriveToGoalCameraConstants.kLatP = SmartDashboard.getNumber("DTG_Cam Lat kP",ConstantValues.DriveToGoalCameraConstants.kLatP)
        ConstantValues.DriveToGoalCameraConstants.kLatD = SmartDashboard.getNumber("DTG_Cam Lat kD",ConstantValues.DriveToGoalCameraConstants.kLatD)
        ConstantValues.DriveToGoalCameraConstants.kLatVmax = SmartDashboard.getNumber("DTG_Cam Lat Vmax",ConstantValues.DriveToGoalCameraConstants.kLatVmax) 
        ConstantValues.DriveToGoalCameraConstants.kLatAmax = SmartDashboard.getNumber("DTG_Cam Lat Amax",ConstantValues.DriveToGoalCameraConstants.kLatAmax)                 
        ConstantValues.DriveToGoalCameraConstants.kRotP = SmartDashboard.getNumber("DTG_Cam Rot kP",ConstantValues.DriveToGoalCameraConstants.kRotP)
        ConstantValues.DriveToGoalCameraConstants.kRotD = SmartDashboard.getNumber("DTG_Cam Rot kD",ConstantValues.DriveToGoalCameraConstants.kRotD)
        ConstantValues.DriveToGoalCameraConstants.kRotVmax = SmartDashboard.getNumber("DTG_Cam Rot Vmax",ConstantValues.DriveToGoalCameraConstants.kRotVmax) 
        ConstantValues.DriveToGoalCameraConstants.kRotAmax = SmartDashboard.getNumber("DTG_Cam Rot Amax",ConstantValues.DriveToGoalCameraConstants.kRotAmax)   


        ## Update values for limelimesystyem
        ConstantValues.LimelightConstants.CAMERA_NAME_R = SmartDashboard.getString("LL Right Cam Name", ConstantValues.LimelightConstants.CAMERA_NAME_R)
        ConstantValues.LimelightConstants.X_OFFSET_R = SmartDashboard.getNumber("LL Right x_offset",ConstantValues.LimelightConstants.X_OFFSET_R) 
        ConstantValues.LimelightConstants.Y_OFFSET_R = SmartDashboard.getNumber("LL Right y_offset",ConstantValues.LimelightConstants.Y_OFFSET_R)
        ConstantValues.LimelightConstants.Z_OFFSET_R = SmartDashboard.getNumber("LL Right z_offset",ConstantValues.LimelightConstants.Z_OFFSET_R) 
        ConstantValues.LimelightConstants.THETA_X_OFFSET_R = SmartDashboard.getNumber("LL Right thetaX_offset",ConstantValues.LimelightConstants.THETA_X_OFFSET_R)
        ConstantValues.LimelightConstants.THETA_Y_OFFSET_R = SmartDashboard.getNumber("LL Right thetaY_offset",ConstantValues.LimelightConstants.THETA_Y_OFFSET_R)
        ConstantValues.LimelightConstants.THETA_Z_OFFSET_R = SmartDashboard.getNumber("LL Right thetaZ_offset",ConstantValues.LimelightConstants.THETA_Z_OFFSET_R)                 
        ConstantValues.LimelightConstants.CAMERA_NAME_L = SmartDashboard.getString("LL Left Cam Name", ConstantValues.LimelightConstants.CAMERA_NAME_L)
        ConstantValues.LimelightConstants.X_OFFSET_L = SmartDashboard.getNumber("LL Left x_offset",ConstantValues.LimelightConstants.X_OFFSET_L) 
        ConstantValues.LimelightConstants.Y_OFFSET_L = SmartDashboard.getNumber("LL Left y_offset",ConstantValues.LimelightConstants.Y_OFFSET_L)
        ConstantValues.LimelightConstants.Z_OFFSET_L = SmartDashboard.getNumber("LL Left z_offset",ConstantValues.LimelightConstants.Z_OFFSET_L) 
        ConstantValues.LimelightConstants.THETA_X_OFFSET_L = SmartDashboard.getNumber("LL Left thetaX_offset",ConstantValues.LimelightConstants.THETA_X_OFFSET_L)
        ConstantValues.LimelightConstants.THETA_Y_OFFSET_L = SmartDashboard.getNumber("LL Left thetaY_offset",ConstantValues.LimelightConstants.THETA_Y_OFFSET_L)
        ConstantValues.LimelightConstants.THETA_Z_OFFSET_L = SmartDashboard.getNumber("LL Left thetaZ_offset",ConstantValues.LimelightConstants.THETA_Z_OFFSET_L)                 
        ConstantValues.LimelightConstants.STD_DEV_COEFF_XY = SmartDashboard.getNumber("LL StdDevCoeff_xy",ConstantValues.LimelightConstants.STD_DEV_COEFF_XY)
        ConstantValues.LimelightConstants.STD_DEV_COEFF_THETA = SmartDashboard.getNumber("LL StdDevCoeff_theta",ConstantValues.LimelightConstants.STD_DEV_COEFF_THETA)
        ConstantValues.LimelightConstants.CAMERA_CUTOFF_DISTANCE = SmartDashboard.getNumber("LL CutoffDist",ConstantValues.LimelightConstants.CAMERA_CUTOFF_DISTANCE)
        
        # Update values for autobuilder
        ConstantValues.AutoBuilderConstants.AUTOBUILDER_XY_kP=  SmartDashboard.getNumber("AutoBuilder XY_kP",ConstantValues.AutoBuilderConstants.AUTOBUILDER_XY_kP)
        ConstantValues.AutoBuilderConstants.AUTOBUILDER_XY_kD=  SmartDashboard.getNumber("AutoBuilder XY_kD",ConstantValues.AutoBuilderConstants.AUTOBUILDER_XY_kD)        
        ConstantValues.AutoBuilderConstants.AUTOBUILDER_THETA_kP=  SmartDashboard.getNumber("AutoBuilder THETA_kP",ConstantValues.AutoBuilderConstants.AUTOBUILDER_THETA_kP)
        ConstantValues.AutoBuilderConstants.AUTOBUILDER_THETA_kD=  SmartDashboard.getNumber("AutoBuilder THETA_kD",ConstantValues.AutoBuilderConstants.AUTOBUILDER_THETA_kD)

        # update values for heading controller
        ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_KP =  SmartDashboard.getNumber("HeadingController kP",ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_KP)
        ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_KD =  SmartDashboard.getNumber("HeadingController kD",ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_KD)
        ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_VMAX =  SmartDashboard.getNumber("HeadingController Vmax",ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_VMAX) 
        ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_AMAX =  SmartDashboard.getNumber("HeadingController Amax",ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_AMAX)                               
        ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_TOLERANCE =  SmartDashboard.getNumber("HeadingController Tol",ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_TOLERANCE)
        ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_RATE_TOLERANCE =  SmartDashboard.getNumber("HeadingController Rate Tol",ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_RATE_TOLERANCE)


 
    @staticmethod
    def write_constants_dashboard():     
        
        SmartDashboard.putNumber("Drive Teleop kP",ConstantValues.DriveConstants.TELEOP_kP)
        SmartDashboard.putNumber("Drive Teleop kS",ConstantValues.DriveConstants.TELEOP_kS) 
        SmartDashboard.putNumber("Drive Teleop kV",ConstantValues.DriveConstants.TELEOP_kV)
        SmartDashboard.putNumber("Drive Teleop kA",ConstantValues.DriveConstants.TELEOP_kA)                           
        SmartDashboard.putNumber("Drive Auto kP",ConstantValues.DriveConstants.AUTO_kP)
        SmartDashboard.putNumber("Drive Auto kS",ConstantValues.DriveConstants.AUTO_kS)
        SmartDashboard.putNumber("Drive Auto kV",ConstantValues.DriveConstants.AUTO_kV)
        SmartDashboard.putNumber("Drive Auto kA",ConstantValues.DriveConstants.AUTO_kA)                        

        SmartDashboard.putNumber("DTG_Cam LatTol",ConstantValues.DriveToGoalCameraConstants.kLateralTolerance)      
        SmartDashboard.putNumber("DTG_Cam ForTol",ConstantValues.DriveToGoalCameraConstants.kForwardTolerance)
        SmartDashboard.putNumber("DTG_Cam RotTol",ConstantValues.DriveToGoalCameraConstants.kRotTolerance)
        SmartDashboard.putNumber("DTG_Cam Debounce",ConstantValues.DriveToGoalCameraConstants.kEndTriggerDebounce)
        SmartDashboard.putNumber("DTG_Cam Timeout Tele",ConstantValues.DriveToGoalCameraConstants.kTimeoutTeleop)
        SmartDashboard.putNumber("DTG_Cam TimeoutAuto",ConstantValues.DriveToGoalCameraConstants.kTimeoutAuto)    
        SmartDashboard.putNumber("DTG_Cam For kP",ConstantValues.DriveToGoalCameraConstants.kForP)
        SmartDashboard.putNumber("DTG_Cam For kD",ConstantValues.DriveToGoalCameraConstants.kForD)
        SmartDashboard.putNumber("DTG_Cam For Vmax",ConstantValues.DriveToGoalCameraConstants.kForVmax) 
        SmartDashboard.putNumber("DTG_Cam For Amax",ConstantValues.DriveToGoalCameraConstants.kForAmax)                 
        SmartDashboard.putNumber("DTG_Cam Lat kP",ConstantValues.DriveToGoalCameraConstants.kLatP)
        SmartDashboard.putNumber("DTG_Cam Lat kD",ConstantValues.DriveToGoalCameraConstants.kLatD)
        SmartDashboard.putNumber("DTG_Cam Lat Vmax",ConstantValues.DriveToGoalCameraConstants.kLatVmax) 
        SmartDashboard.putNumber("DTG_Cam Lat Amax",ConstantValues.DriveToGoalCameraConstants.kLatAmax)                 
        SmartDashboard.putNumber("DTG_Cam Rot kP",ConstantValues.DriveToGoalCameraConstants.kRotP)
        SmartDashboard.putNumber("DTG_Cam Rot kD",ConstantValues.DriveToGoalCameraConstants.kRotD)
        SmartDashboard.putNumber("DTG_Cam Rot Vmax",ConstantValues.DriveToGoalCameraConstants.kRotVmax) 
        SmartDashboard.putNumber("DTG_Cam Rot Amax",ConstantValues.DriveToGoalCameraConstants.kRotAmax) 


        SmartDashboard.putString("LL Right Cam Name", ConstantValues.LimelightConstants.CAMERA_NAME_R)
        SmartDashboard.putNumber("LL Right x_offset",ConstantValues.LimelightConstants.X_OFFSET_R) 
        SmartDashboard.putNumber("LL Right y_offset",ConstantValues.LimelightConstants.Y_OFFSET_R)
        SmartDashboard.putNumber("LL Right z_offset",ConstantValues.LimelightConstants.Z_OFFSET_R) 
        SmartDashboard.putNumber("LL Right thetaX_offset",ConstantValues.LimelightConstants.THETA_X_OFFSET_R)
        SmartDashboard.putNumber("LL Right thetaY_offset",ConstantValues.LimelightConstants.THETA_Y_OFFSET_R)
        SmartDashboard.putNumber("LL Right thetaZ_offset",ConstantValues.LimelightConstants.THETA_Z_OFFSET_R)                 
        SmartDashboard.putString("LL Left Cam Name", ConstantValues.LimelightConstants.CAMERA_NAME_L)
        SmartDashboard.putNumber("LL Left x_offset",ConstantValues.LimelightConstants.X_OFFSET_L) 
        SmartDashboard.putNumber("LL Left y_offset",ConstantValues.LimelightConstants.Y_OFFSET_L)
        SmartDashboard.putNumber("LL Left z_offset",ConstantValues.LimelightConstants.Z_OFFSET_L) 
        SmartDashboard.putNumber("LL Left thetaX_offset",ConstantValues.LimelightConstants.THETA_X_OFFSET_L)
        SmartDashboard.putNumber("LL Left thetaY_offset",ConstantValues.LimelightConstants.THETA_Y_OFFSET_L)
        SmartDashboard.putNumber("LL Left thetaZ_offset",ConstantValues.LimelightConstants.THETA_Z_OFFSET_L)                 
        SmartDashboard.putNumber("LL StdDevCoeff_xy",ConstantValues.LimelightConstants.STD_DEV_COEFF_XY)
        SmartDashboard.putNumber("LL StdDevCoeff_theta",ConstantValues.LimelightConstants.STD_DEV_COEFF_THETA)
        SmartDashboard.putNumber("LL CutoffDist",ConstantValues.LimelightConstants.CAMERA_CUTOFF_DISTANCE)


        SmartDashboard.putNumber("AutoBuilder XY_kP",ConstantValues.AutoBuilderConstants.AUTOBUILDER_XY_kP)
        SmartDashboard.putNumber("AutoBuilder XY_kD",ConstantValues.AutoBuilderConstants.AUTOBUILDER_XY_kD)
        SmartDashboard.putNumber("AutoBuilder THETA_kP",ConstantValues.AutoBuilderConstants.AUTOBUILDER_THETA_kP)
        SmartDashboard.putNumber("AutoBuilder THETA_kD",ConstantValues.AutoBuilderConstants.AUTOBUILDER_THETA_kD)


        SmartDashboard.putNumber("HeadingController kP",ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_KP)
        SmartDashboard.putNumber("HeadingController kD",ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_KD)
        SmartDashboard.putNumber("HeadingController Vmax",ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_VMAX) 
        SmartDashboard.putNumber("HeadingController Amax",ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_AMAX)                               
        SmartDashboard.putNumber("HeadingController Tol",ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_TOLERANCE)
        SmartDashboard.putNumber("HeadingController Rate Tol",ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_RATE_TOLERANCE)


