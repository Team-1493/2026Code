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
from telemetry import Telemetry
from phoenix6 import swerve

from wpilib import DriverStation
from wpilib import SmartDashboard

from generated.tuner_constants import TunerConstants
from subsystems.Vision.LLSystem import llSystem
from subsystems.Vision.PhotonVisionSim import PVisionSim
from subsystems.Drive.driveTrainGenerate import DrivetrainGenerator

from subsystems.Drive.headingcontroller import HeadingController

from Commands.DrivePathGenerator import DrivePath

from Commands.Drivecommand import DriveTeleopCommand
from Auto.AutoCommandGenerator import AutoGenerator
from pathplannerlib.auto import PathPlannerAuto
from pathplannerlib.auto import AutoBuilder

class RobotContainer:

    def __init__(self) -> None:
        self.drivetrain = DrivetrainGenerator.getInstance()
        self.headingController = HeadingController.getInstance()
        self.limelightSytem = llSystem()
        self.pvSim = PVisionSim()
        self._joystick = CommandXboxController(0)

        self.autoGenerator = AutoGenerator()
        self.autoChooser = AutoBuilder.buildAutoChooser("Auto1")
        SmartDashboard.putData("Auto Chooser", self.autoChooser)
        
        self._logger = Telemetry(TunerConstants.speed_at_12_volts)

        
        # speed_at_12_volts desired top speed
        self._max_speed = (TunerConstants.speed_at_12_volts)  
        
        self.drivePath = DrivePath(
                lambda: self.drivetrain.get_state().pose)
        

        self.driveTeleopCommand = DriveTeleopCommand(self.drivetrain,
                lambda: -self._joystick.getRawAxis(1),
                lambda: -self._joystick.getRawAxis(0),
                lambda: -self._joystick.getRawAxis(4))
    
        self.a :bool

        # Configure the button bindings
        self.configureButtonBindings()

    def configureButtonBindings(self) -> None:


        self.drivetrain.setDefaultCommand(self.driveTeleopCommand)

       
        # Idle while the robot is disabled. This ensures the configured
        # neutral mode is applied to the drive motors while disabled.
        idle = swerve.requests.Idle()
        Trigger(DriverStation.isDisabled).whileTrue(
            self.drivetrain.apply_request(lambda: idle).ignoringDisable(True)
        )


        # reset the field-centric heading on left bumper press
        self._joystick.leftBumper().onTrue(
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())
        )

        
        self._joystick.button(1).onTrue(
            self.headingController.runOnce(lambda:self.headingController.rotateToZero()))
        
        self._joystick.button(2).onTrue(
            self.headingController.runOnce(lambda:self.headingController.rotateTo90()))

        self._joystick.button(3).onTrue(
            self.headingController.runOnce(lambda:self.headingController.rotateTo180()))

        self._joystick.button(7).onTrue(
            self.headingController.runOnce(lambda:self.headingController.rotateTo270()))

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

        self._joystick.button(4).whileTrue(
            commands2.DeferredCommand(lambda:self.drivePath.drivePathToTag(2,-.5,.8)).finallyDo
            (self.headingController.setTargetRotationInt))
        
        self._joystick.button(8).whileTrue(
            commands2.DeferredCommand(lambda:self.drivePath.drivePathFindToTag(2,-.5,.8)).finallyDo
            (self.headingController.setTargetRotationInt))        


    def getAutonomousCommand(self):
        return self.autoChooser.getSelected()
    
    def setHeadingControlToCurrentrHeading(self):
        self.headingController.setTargetRotationInt(True)    