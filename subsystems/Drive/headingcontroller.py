from commands2 import Command, InstantCommand, Subsystem
from wpilib import SmartDashboard
import commands2
from wpimath.controller  import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile
from math import pi,hypot
from subsystems.Drive.driveTrainGenerate import DrivetrainGenerator 

#To Do - check if rotation not being controlled until first rotation

class HeadingController(Subsystem):
    instance=None
    
    @staticmethod
    def getInstance():
        if HeadingController.instance == None:
            HeadingController.instance = HeadingController()
            print("**********************  HEADING CONTROLLER  **********************") 
        return HeadingController.instance
    
    def __init__(self):

        self.state =False # false = not auto-rotating to target angle, true = rotating to target angle

        self.driveTrain = DrivetrainGenerator.getInstance()

        self.rotation = 0.0
        self.rotationRate = 0.0
        self.targetRotation = 0.0
        self.targetRotationRate = 0.0        
        self.targetRotationPrev=0.0

        self.tp = TrapezoidProfile.Constraints(2, 4) #1,2
        self.headingController = ProfiledPIDController(16, 0, 0,self.tp) #8

        self.headingController.enableContinuousInput(-pi, pi)
        self.headingController.setTolerance(0.017);  #1 degree
        # Need to tune this carefullly - balance between having setpoint drift 
        # and not complating an auto rotation
        self.headingRateTolerance = .3 # measured rotation rate at which we assume robot not rotating
 
        self.headingController.setGoal(self.targetRotation); 


    def calulateRotationRate(self,stick_rot):
        self.rotation = self.getRotation()
        self.rotationRate = self.getRotationRate()
        speed = self.getSpeed()
        a=stick_rot

        if abs(stick_rot) > 0 or (abs(self.rotationRate)>self.headingRateTolerance and not self.state):
            self.targetRotationPrev = self.targetRotation
            self.targetRotation = self.rotation
            self.state=False

        elif self.state or  speed>0.3 :
            if self.targetRotation != self.targetRotationPrev:
                self.targetRotationPrev=self.targetRotation
                self.setTargetRotation(self.targetRotation)               
            stick_rot = self.headingController.calculate(self.rotation)

            if self.headingController.atSetpoint(): 
                stick_rot = 0.0
                self.state=False
        
#        print("stick_rot I: ",f"{a:.4f}", "   rot: ", f"{self.rotation:.4f}", "   Rot Rate: ",f"{self.rotationRate:.4f}", 
#              "   rot_T: ",f"{self.targetRotation:.4f}", "   state: ", self.state, "   stick_rot F: ",f"{stick_rot:.4f}" ) 
        SmartDashboard.putNumber("controller at SP",self.headingController.atSetpoint())
        SmartDashboard.putNumber("controller SP",self.headingController.getSetpoint().position*180/3.14)
        SmartDashboard.putNumber("controller err",self.headingController.getPositionError()*180/3.14)
        SmartDashboard.putNumber("stick rot",stick_rot)        
        return stick_rot


    def setTargetRotation(self,angle : float):
        self.headingController.reset(angle)
        self.targetRotation = angle
        self.targetRotationPrev = self.targetRotation
        self.headingController.setGoal(self.targetRotation)


    def rotateToZero(self):
        self.setTargetRotation(0)
        self.state=True

    def rotateTo90(self):
        self.setTargetRotation(pi/2)
        self.state=True

    def rotateTo180(self):
        self.setTargetRotation(pi)
        self.state=True

    def rotateTo270(self):
        self.setTargetRotation(3*pi/2)
        self.state=True        

    def getRotation(self) -> float:
        return self.driveTrain.get_state().pose.rotation().radians()
    
    def getRotationRate(self) -> float:
        return self.driveTrain.get_state().speeds.omega
    
    def getSpeed(self) -> float:
        return hypot(self.driveTrain.get_state().speeds.vx,self.driveTrain.get_state().speeds.vy) 

    def setTargetRotationCommand(self,angle) -> Command:
        return  InstantCommand(lambda: self.setTargetRotation(angle))   
    
    def setTargetRotationInt (self,b:bool):
        self.setTargetRotation(self.getRotation())
        
    
    
