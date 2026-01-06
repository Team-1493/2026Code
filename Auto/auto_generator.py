from commands2 import Command
from wpimath.geometry import Pose2d, Rotation2d
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants
from wpilib import DriverStation
from Commands.stop_drive import StopDrive
from robot_state import RobotState
from subsystems.Drive.drivetrain_generator import DrivetrainGenerator
from phoenix6 import swerve
from Constants1 import ConstantValues
from Commands.goal_cam_command import GoalCamCommand
from wpimath.kinematics import ChassisSpeeds
from pathplannerlib.auto import NamedCommands


class AutoGenerator():
    

    def __init__(self):
        self.constants = ConstantValues.AutoBuilderConstants
        self.robotState = RobotState.getInstance()
        self.driveTrain = DrivetrainGenerator.getInstance()
        self._apply_robot_speeds = swerve.requests.ApplyRobotSpeeds()
        self.driveRC=  swerve.requests.ApplyRobotSpeeds().with_drive_request_type(
            swerve.SwerveModule.DriveRequestType.VELOCITY)
        self.configAutoBuilder()
        self.create_named_commands()



    def create_named_commands(self):
        NamedCommands.registerCommand('DriveToGoalCam', GoalCamCommand(-1,0))
        NamedCommands.registerCommand('StopDrive', StopDrive())        


    def configAutoBuilder(self):
        autoXY_kP = self.constants.AUTOBUILDER_XY_kP
        autoXY_kD = self.constants.AUTOBUILDER_XY_kD
        autoTHETA_kP = self.constants.AUTOBUILDER_THETA_kP
        autoTHETA_kD = self.constants.AUTOBUILDER_THETA_kD
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
                PIDConstants(autoXY_kP, 0.0, autoXY_kD), # Translation PID constants
                PIDConstants(autoTHETA_kP, 0.0, autoTHETA_kD) # Rotation PID constants
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



