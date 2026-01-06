import typing
import commands2
from math import copysign
from subsystems.Drive.drivetrain_generator import DrivetrainGenerator
from generated.tuner_constants import TunerConstants
from subsystems.Drive.heading_controller import HeadingController
from phoenix6 import swerve
from wpimath.units import rotationsToRadians
from wpilib import SmartDashboard
from  subsystems.Drive.command_swerve_drivetrain import CommandSwerveDrivetrain

class DriveTeleopCommand(commands2.Command):
    def __init__(
        self,
        _drivetrain: CommandSwerveDrivetrain,
        forward: typing.Callable[[], float],
        side: typing.Callable[[], float],
        rotate: typing.Callable[[], float],
    ) -> None:
        super().__init__()
        self.forward = forward
        self.side = side
        self.rotate = rotate

        self.drivetrain = _drivetrain

        self.headingController = HeadingController.getInstance()

        # speed_at_12_volts desired top speed
        self._max_speed = (TunerConstants.speed_at_12_volts)  

        # 3/4 of a rotation per second max angular velocity
        self._max_angular_rate = rotationsToRadians(0.75)  

        # Setting up bindings for necessary control of the swerve drive platform
        self.requestFC = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.0025)  #squared input, so db starts at 0.05
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.VELOCITY
            ))
        

      
        self.addRequirements(self.drivetrain)


    def execute(self) -> None:
        forw=self.forward()
        sde=self.side()
        rot = self.rotate()
        if rot<0.05 and rot>-0.05: rot=0
        if forw<0.05 and forw>-0.05: forw=0
        if sde<0.05 and sde>-0.05: sde=0                
        # square input
        forw = copysign(forw**2,forw)
        sde = copysign(sde**2,sde)
        rot = copysign(rot**2,rot)

        rot = self.headingController.calulateRotationRate(rot*self._max_angular_rate)
        if rot<-self._max_angular_rate: rot=-self._max_angular_rate
        elif  rot>self._max_angular_rate: rot=self._max_angular_rate
        self.drivetrain.set_control(
                self.requestFC.
                with_velocity_x(forw*self._max_speed).
                with_velocity_y(sde*self._max_speed).
                with_rotational_rate(rot))

        
