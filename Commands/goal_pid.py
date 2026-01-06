from typing import override
import commands2
from pathplannerlib.config  import PIDConstants
from pathplannerlib.controller import PPHolonomicDriveController 
from pathplannerlib.trajectory import PathPlannerTrajectoryState;
from wpilib import Timer
from math import hypot, pi
from wpimath.geometry import Pose2d,Rotation2d
from wpimath.kinematics import ChassisSpeeds
from phoenix6 import swerve

from commands2 import Command
from commands2.button import Trigger
from subsystems.Drive.drivetrain_generator import DrivetrainGenerator
from  subsystems.Drive.command_swerve_drivetrain import CommandSwerveDrivetrain




class GoalPID(commands2.Command):
    def __init__(self, goalPose: Pose2d):
        self.driveTrain = DrivetrainGenerator.getInstance()
        self.goalPose = goalPose
        self.kTranslationPID = PIDConstants(5.0,0,0)
        self.kRotationPID = PIDConstants(5.0,0,0)
        self.kRotationTolerance = Rotation2d.fromDegrees(2.0)
        self.kPositionTolerance = 0.02 # meters
        self.kSpeedTolerance = 0.0254  # meters per second, equal to 1 inch per second    
        self.kEndTriggerDebounce = 0.1 #seconds
        self.kTeleopAlignAdjustTimeout = 2 #seconds
        self.kAutoAlignAdjustTimeout = 0.6 #seconds
        self.controller = PPHolonomicDriveController(
            self.kTranslationPID, self.kRotationPID)
        
        self.timer =Timer()
        self.endTriggerDebounced: Trigger
        self.requestRC = (swerve.requests.RobotCentric().with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.VELOCITY))

    @override
    def initialize(self):
        self.timer.restart()
        self.endTriggerDebounced = self.get_end_trigger()
    

    @override
    def execute(self): 
        goalState = PathPlannerTrajectoryState()
        goalState.pose = self.goalPose

        speeds =self.controller.calculateRobotRelativeSpeeds(
            self.driveTrain.get_state().pose, goalState)

        speeds = ChassisSpeeds(min(speeds.vx, 1.0),min(speeds.vy, 1.0),
                speeds.omega)

        self.driveRC(speeds.vx,speeds.vy,speeds.omega)


    @override
    def end(self, interrupted: bool) :
        self.timer.stop()
        self.driveRC(0,0,0)

    @override
    def isFinished(self):
        return self.endTriggerDebounced.getAsBoolean()
    


    def isNear(self,  expected: float,  actual: float, 
               tolerance: float, min: float, max: float): 

        if tolerance < 0 :
            tolerance = abs(tolerance)

        # Max error is exactly halfway between the min and max
        errorBound = (max - min) / 2.0
        error = self.inputModulus(expected - actual, -errorBound, errorBound)
        return abs(error) < tolerance
  



    
    def inputModulus(self, input : float,  minimumInput : float,  maximumInput: float):
        modulus = maximumInput - minimumInput
        
        # Wrap input if it's above the maximum input
        numMax = int ((input - minimumInput) / modulus)
        input = input - numMax * modulus

        #Wrap input if it's below the minimum input
        numMin = int ((input - maximumInput) / modulus)
        input = input -  numMin * modulus

        return input
    

    def get_end_trigger(self):
        def condition():
            state = self.driveTrain.get_state()
            diff = (state.pose).relativeTo(self.goalPose)

            position = diff.translation().norm() < self.kPositionTolerance

            rotation = self.isNear(
                0.0,
                diff.rotation().radians(),
                self.kRotationTolerance.radians(),
                -pi, pi,)

            speed = (hypot(state.speeds.vx,state.speeds.vy) < self.kSpeedTolerance)
            return position and rotation and speed
           
        return ((Trigger(condition)).debounce(self.kEndTriggerDebounce))
    

    def driveRC(self, x,y,z):
        self.driveTrain.set_control(
            self.requestRC.with_velocity_x(x)
            .with_velocity_y(y)
            .with_rotational_rate(z)) 

    