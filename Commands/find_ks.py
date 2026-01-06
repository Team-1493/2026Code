import math
from typing import override
import commands2
from wpilib import SmartDashboard, Timer
import wpimath
from subsystems.Drive.drivetrain_generator import DrivetrainGenerator
from phoenix6 import swerve
from  subsystems.Drive.command_swerve_drivetrain import CommandSwerveDrivetrain

class FindkS(commands2.Command):
    def __init__(self):
        self.drivetrain = DrivetrainGenerator.getInstance()
        self.driveKS =swerve.requests.SysIdSwerveTranslation()
      
        self.addRequirements(self.drivetrain)
        self.timer = Timer()
        self.voltageRate = 0.01  #ramp rate volts per second
    

    
    @override
    def initialize(self):
        self.timeCurrent = 0
        self.voltage = 0
        self.timer.reset()
        self.timer.start()
        self.timePrevious = 0
        self.kS = 0
        SmartDashboard.putNumber("Drive kS - measured",0)
        self.haskS = False

    def execute(self) -> None:
        timeCurrent = self.timer.get()
        deltaTime = timeCurrent - self.timePrevious
        self.voltage += deltaTime*self.voltageRate

        self.drivetrain.set_control(self.driveKS.with_volts(self.voltage))
        chassisSpeed = self.drivetrain.get_state().speeds
        spd = math.sqrt(chassisSpeed.vx**2+chassisSpeed.vy**2)
        if spd>.005:
            self.haskS = True

        if not self.haskS:
            SmartDashboard.putNumber("Drive kS - measured",self.voltage)

    @override
    def end(self, interrupted: bool) :
        self.drivetrain.set_control(self.driveKS.with_volts(0))

    @override
    def isFinished(self):
        return False
    

        
