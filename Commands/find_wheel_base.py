import math
from typing import override
import commands2
from wpilib import SmartDashboard, Timer
from subsystems.Drive.drivetrain_generator import DrivetrainGenerator
from phoenix6 import swerve
from  subsystems.Drive.command_swerve_drivetrain import CommandSwerveDrivetrain
from wpimath.geometry import Pose2d,Rotation2d

class FindWheelBase(commands2.Command):
    def __init__(self):
        self.drivetrain = DrivetrainGenerator.getInstance()
        self.m_request = (swerve.requests.FieldCentricFacingAngle()
                .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
                .with_heading_pid(4,0,0))
        self.addRequirements(self.drivetrain)
        self.timer = Timer()
        self.rotationRate = 0.5  #ramp rate volts per second
    

    
    @override
    def initialize(self):
        self.timeCurrent = 0
        self.rotation = 0
        self.timer.reset()
        self.timer.start()
        self.timePrevious = 0
        self.halfway = False

    def execute(self) -> None:
        timeCurrent = self.timer.get()
        deltaTime = timeCurrent - self.timePrevious
        self.timePrevious = timeCurrent
        self.rotation+= deltaTime*self.rotationRate
        print(deltaTime,self.rotation)
        if self.rotation>2*math.pi:
            self.rotation = 2*math.pi
        rot = Rotation2d(self.rotation)

        self.drivetrain.set_control(
            self.m_request.with_target_direction(rot))


    

    @override
    def end(self, interrupted: bool) :
        self.drivetrain.set_control(self.m_request.with_velocity_x(0)
                                    .with_velocity_y(0))

    @override
    def isFinished(self):
        return False
    

        
