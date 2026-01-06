from typing import override
import commands2
from wpilib import SmartDashboard, Timer
from math import hypot, pi
from wpimath.geometry import Pose2d,Rotation2d
from wpimath.kinematics import ChassisSpeeds
from phoenix6 import swerve
from wpimath.controller  import ProfiledPIDController,PIDController
from wpimath.trajectory import TrapezoidProfile
from commands2 import Command
from commands2.button import Trigger
from subsystems.Drive.drivetrain_generator import DrivetrainGenerator
from  subsystems.Drive.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.laser_can import LaserCAN
from Utilities.LLH import LimelightHelpers
from Constants1 import ConstantValues


class GoalCamCommand(commands2.Command):
    def __init__(self, goalFor:float, goalLat:float, id:int = 0):
        self.constants = ConstantValues.DriveToGoalCameraConstants
        self.constantsVision = ConstantValues.VisionConstants
        self.leftCamName = ConstantValues.LimelightConstants.CAMERA_NAME_L
        self.rightCamName = ConstantValues.LimelightConstants.CAMERA_NAME_R        
        self.driveTrain = DrivetrainGenerator.getInstance()        
        self.timer =Timer()
        self.angle = 0
        self.rot = 0
        self.endTriggerDebounced: Trigger
        self.requestRC = (swerve.requests.RobotCentric().with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.VELOCITY))
        
        self.LC = LaserCAN.getInstance()
        self.llh = LimelightHelpers
        self.goalFor = -goalFor
        self.goalLat = goalLat     
        self.id = id 

        self.setup()



    @override
    def initialize(self):
        self.id = 0
        # reset controllers
        self.controllerRot.reset() 
        self.controllerFor.reset()
        self.controllerLat.reset() 

        #  If no tag ID sent, use the closest visible ID  
        if self.id == 0:
            self.id = int(self.llh.get_fiducial_id(self.leftCamName))

        # if we have a valid tag ID, use that tag's rotation as the target rotation
        # if not, use the current robot rotatation as the target (maintain current rotation)
        if self.id >0:
            self.goalRot = self.constantsVision.tags_list[self.id-1].pose.toPose2d().rotation().radians()-pi
        else: 
            pose = self.driveTrain.get_state().pose
            self.goalRot = pose.rotation().radians()

        # adjust for wraparound
        if self.goalRot<-pi: self.goalRot+=2*pi
        if self.goalRot>pi: self.goalRot-=2*pi

        # set controller setppoints, forward an lateral goals
        self.controllerFor.setSetpoint(self.goalFor)
        self.controllerLat.setSetpoint(self.goalLat)  
        self.controllerRot.setSetpoint(self.goalRot) 

        #get the end trigger and start timer 
        self.endTriggerDebounced = self.get_end_trigger()
        self.timer.reset() 
        self.timer.start()      

        print("Goals: ",self.id,self.goalFor,self.goalLat,self.goalRot) 
        
    

    @override
    def execute(self):
        #Calculate vx, vy and omega. If no tag is visible vy = 0, 
        # if no lasercan data vx = 0

        # get yaw and calculate vy if tag is visible, else set vy = 0
        
        if self.llh.get_tv(self.leftCamName): 
            self.angle = self.llh.get_tx(self.leftCamName)
            vy = self.controllerLat.calculate(self.angle)
            vy = self.cap(vy,self.kLatVmax)
        else:
            self.angle = 0
            vy = 0

        # get distance from lasercam and calculate vx id measurement is valid,  set vx = 0        
        self.dist = self.LC.get_distance_meters()          
        if self.LC.status==0:  # status = 0 for valid measurement
            vx = -self.controllerFor.calculate(self.dist)
            vx = self.cap(vx,self.kForVmax) 
        else:
            vx = 0

        # get robot current rotation and calculate omega
        self.rot = self.driveTrain.get_state().pose.rotation().radians()
        omega = self.controllerRot.calculate(self.rot)
        omega = self.cap(omega,self.kRotVmax)                          
                 
        print("tv: ",self.llh.get_tv(self.leftCamName),
              "   dist: ",round(self.dist,3),
              "   tx: ",round(self.angle,3),
              "   rot: ",round(self.rot,3),              
              "   vx: ",round(vx,3),
              "   vy: ",round(vy,3),
              "   om: ",round(omega,3)              
            )

        self.driveRC(vx,vy,omega)


    @override
    def end(self, interrupted: bool) :
        self.timer.stop()
        print("Time:  ",self.timer.get())
        self.driveRC(0,0,0)

    @override
    def isFinished(self):
        return self.endTriggerDebounced.getAsBoolean()
    
    

    def get_end_trigger(self):
        def condition():
            forOK = self.controllerFor.atSetpoint()
            latOK = self.controllerLat.atSetpoint()
            rotOK = self.controllerRot.atSetpoint()                        
            return (forOK and latOK and rotOK) or self.timer.get()>self.kTimeoutTeleop
           
        return ((Trigger(condition)).debounce(self.kEndTriggerDebounce))
    

    def driveRC(self, x,y,z):
        self.driveTrain.set_control(
            self.requestRC.with_velocity_x(x)
            .with_velocity_y(y)
            .with_rotational_rate(z)) 

    # enforce maximum value of control effort
    def cap(self,value,minmax):
        if value>minmax: value=minmax
        if value<-minmax: value = -minmax
        return value

    # set up the three PID controllers using values from the constants file
    def setup(self):
        self.kLateralTolerance = self.constants.kLateralTolerance
        self.kForwardTolerance = self.constants.kForwardTolerance # meters
        self.kRotTolerance = self.constants.kRotTolerance
        self.kEndTriggerDebounce = self.constants.kEndTriggerDebounce #seconds
        self.kTimeoutTeleop = self.constants.kTimeoutTeleop #seconds
        self.kTimeoutAuto = self.constants.kTimeoutAuto #seconds
        self.kForVmax = self.constants.kForVmax
        self.kForAmax = self.constants.kForAmax
        self.kForP = self.constants.kForP                
        self.kForD = self.constants.kForD
        self.kLatVmax = self.constants.kLatVmax
        self.kLatAmax =self.constants.kLatAmax
        self.kLatP = self.constants.kLatP               
        self.kLatD = self.constants.kLatD
        self.kRotVmax = self.constants.kRotVmax
        self.kRotAmax = self.constants.kRotAmax
        self.kRotP = self.constants.kRotP
        self.kRotD = self.constants.kRotD


        self.tpFor = TrapezoidProfile.Constraints(self.kForVmax, self.kForAmax) #1,2
#        self.controllerFor = ProfiledPIDController(self.kForP, 0, self.kForD,self.tpFor) #8
        self.controllerFor = PIDController(self.kForP, 0, self.kForD) #8        
        self.controllerFor.setTolerance(self.kForwardTolerance);  

        print("vmax = ",self.kLatVmax,"amax = ",self.kLatAmax)
        self.tpLat = TrapezoidProfile.Constraints(self.kLatVmax, self.kLatAmax) #1,2
        self.controllerLat = PIDController(self.kLatP , 0, self.kLatD) #8
        self.controllerLat.setTolerance(self.kLateralTolerance)

        self.tpRot = TrapezoidProfile.Constraints(self.kRotVmax, self.kRotAmax) #1,2
        self.controllerRot = PIDController(self.kRotP, 0, self.kRotD) #12
        self.controllerRot.enableContinuousInput(-pi, pi)
        self.controllerRot.setTolerance(self.kLateralTolerance);  #1 degree


  


        
        
    