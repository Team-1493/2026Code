#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import commands2.cmd
from commands2.button import CommandXboxController, Trigger
from wpilib import Joystick
from wpilib import XboxController
from commands2.sysid import SysIdRoutine
from AutoPilot_py import APTarget
from Commands.auto_pilot_command import AutoPilotCommand
from Commands.find_wheel_base import FindWheelBase
from Commands.find_ks import FindkS
from telemetry import Telemetry
from phoenix6 import swerve
from wpimath.geometry import Pose2d, Rotation2d,Translation2d
from wpilib import DriverStation
from wpilib import SmartDashboard

from Constants1 import ConstantValues
from generated.tuner_constants import TunerConstants
from subsystems.Vision.limelight_system import LLsystem
from subsystems.Vision.photon_vision_sim import PVisionSim
from subsystems.Drive.drivetrain_generator import DrivetrainGenerator
from subsystems.Drive.heading_controller import HeadingController
from Commands.drive_path_generator import DrivePathGenerator

from Commands.drive_teleop_command import DriveTeleopCommand
from Auto.auto_generator import AutoGenerator
from pathplannerlib.auto import PathPlannerAuto
from pathplannerlib.auto import AutoBuilder
from wpilib import DataLogManager


class RobotContainer:

    def __init__(self) -> None:
        self.drivetrain = DrivetrainGenerator.getInstance()
        self.constants = ConstantValues.getInstance()
        self.headingController = HeadingController.getInstance()
        self.limelightSytem = LLsystem.getInstance()
        self.pvSim = PVisionSim()
        self._joystick = CommandXboxController(0)

        self.autoGenerator = AutoGenerator()
        self.autoChooser = AutoBuilder.buildAutoChooser("Auto1")
        SmartDashboard.putData("Auto Chooser", self.autoChooser)
        
        self._logger = Telemetry(TunerConstants.speed_at_12_volts)
        DataLogManager.start()
        
        # speed_at_12_volts desired top speed
        self._max_speed = (TunerConstants.speed_at_12_volts)  
        
        self.drive_path = DrivePathGenerator(
                lambda: self.drivetrain.get_state().pose)
        
        self.drive_teleop_command = DriveTeleopCommand(self.drivetrain,
                lambda: -self._joystick.getRawAxis(1),
                lambda: -self._joystick.getRawAxis(0),
                lambda: -self._joystick.getRawAxis(4))
    

        # Configure the button bindings
        self.configureButtonBindings()

    def configureButtonBindings(self) -> None:


        self.drivetrain.setDefaultCommand(self.drive_teleop_command)

       
        # Idle while the robot is disabled. This ensures the configured
        # neutral mode is applied to the drive motors while disabled.
        idle = swerve.requests.Idle()
        Trigger(DriverStation.isDisabled).whileTrue(
            self.drivetrain.apply_request(lambda: idle).ignoringDisable(True)
        )


        # reset the field-centric heading on left bumper press
        """
        self._joystick.button(5).onTrue(
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())
        )
        """

        
        self._joystick.button(1).onTrue(
            self.headingController.runOnce(lambda:self.headingController.rotateToZero()))
        
        self._joystick.button(2).onTrue(
            self.headingController.runOnce(lambda:self.headingController.rotateTo90()))

        self._joystick.button(3).onTrue(
            self.headingController.runOnce(lambda:self.headingController.rotateTo180()))

        self._joystick.button(4).onTrue(
            self.headingController.runOnce(lambda:self.headingController.rotateTo270()))

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

        self._joystick.button(5).whileTrue(
            FindWheelBase().finallyDo((self.headingController.setTargetRotationInt)))

#        self._joystick.button(7).whileTrue(
#            commands2.DeferredCommand(lambda:self.drive_path.drive_pathfind_to_tag(16,-1,0)).finallyDo
#           (self.headingController.setTargetRotationInt))
        
        self._joystick.button(7).whileTrue(
            commands2.DeferredCommand(lambda:self.drive_path.drive_path_to_tag(16,-1,0)).finallyDo
            (self.headingController.setTargetRotationInt))
        
    
#        self._joystick.button(8).whileTrue(
#            commands2.DeferredCommand(lambda:self.drive_path.drive_path_to_tag(17,-1,0)).finallyDo
#           (self.headingController.setTargetRotationInt))

        self._joystick.button(8).whileTrue(
            AutoPilotCommand(17,-1,0,0 ).finallyDo((self.headingController.setTargetRotationInt)))
 

        self._joystick.button(9).onTrue(
              commands2.InstantCommand(lambda:self.update_constants()))





    def getAutonomousCommand(self):
        return self.autoChooser.getSelected()
    
    def setHeadingControlToCurrentrHeading(self):
        self.headingController.setTargetRotationInt(True)  

    def update_constants(self):
        # transfer constants from smartdashbaord to constants class        
        self.constants.update_constants()
        # update limelight, autobuilder, and heading controller constants  
        self.limelightSytem.configfureLimelights()
        self.autoGenerator.configAutoBuilder()
        self.headingController.setup()
        DrivetrainGenerator.updateGains()
        DrivetrainGenerator.apply_teleop_gains()
        # DriveGoal_Cam does not need to be explicitly updated, it is generated at each use
