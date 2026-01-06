import commands2
from subsystems.Drive.drivetrain_generator import DrivetrainGenerator
from phoenix6 import swerve
from  subsystems.Drive.command_swerve_drivetrain import CommandSwerveDrivetrain

class StopDrive(commands2.Command):
    def __init__(self):
 
        self.drivetrain = DrivetrainGenerator.getInstance()

        self.stop = (
            swerve.requests.FieldCentric().with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_velocity_x(0).with_velocity_y(0).with_rotational_rate(0)
            )
      


    def execute(self) -> None:
        self.drivetrain.set_control(self.stop)

        
