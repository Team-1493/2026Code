from commands2 import Command
from wpimath.geometry import Pose2d, Rotation2d
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants
from wpilib import DriverStation
from robotstate import RobotState
from subsystems.Drive.driveTrainGenerate import DrivetrainGenerator
from phoenix6 import swerve

from wpimath.kinematics import ChassisSpeeds


class AutoGenerator():

    def __init__(self):
        self.robotState = RobotState.getInstance()
        self.driveTrain = DrivetrainGenerator.getInstance()
        self.configAutoBuilder()
        self._apply_robot_speeds = swerve.requests.ApplyRobotSpeeds()
        self.driveRC=  swerve.requests.ApplyRobotSpeeds().with_drive_request_type(
            swerve.SwerveModule.DriveRequestType.VELOCITY)




    def configAutoBuilder(self):
        config = RobotConfig.fromGUISettings()

        AutoBuilder.configure(
            self.getPose, # Robot pose supplier
            self.driveTrain.reset_pose, # Method to reset odometry (will be called if your auto has a starting pose)
            self.getSpeeds, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
#            lambda speeds, feedforwards: self.driveRobotRelative(speeds), # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also outputs individual module feedforwards
            
            lambda speeds, feedforwards: self.driveTrain.set_control(
                self.driveRC.with_speeds(speeds)
                ),
            PPHolonomicDriveController( # PPHolonomicController is the built in path following controller for holonomic drive trains
                PIDConstants(7.0, 0.0, 0.0), # Translation PID constants
                PIDConstants(7.0, 0.0, 0.0) # Rotation PID constants
            ),
            config, # The robot configuration
            self.shouldFlipPath, # Supplier to control path flipping based on alliance color
            self.driveTrain # Reference to this subsystem to set requirements
    )
        

    def shouldFlipPath(self):
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed
    

    def getPose(self):
        return self.driveTrain.get_state().pose
    
    def getSpeeds(self):
        return self.driveTrain.get_state().speeds    



