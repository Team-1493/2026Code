
from subsystems.Drive.driveTrainGenerate import DrivetrainGenerator 
from math import hypot
from wpimath.geometry import Pose2d
from wpilib import SmartDashboard
#import edu.wpi.first.math.kinematics.ChassisSpeeds;
#import com.team2052.lib.helpers.MathHelpers;



class RobotState():
    instance=None
 

    @staticmethod
    def getInstance():
        if RobotState.instance == None:
            RobotState.instance = RobotState()
            print("**********************  CREATING ROBOTSTATE  **********************") 
        return RobotState.instance
    

    def __init__(self) -> None:
        self.driveTrain=DrivetrainGenerator.getInstance()
        self.driveTrainState = self.driveTrain.get_state()
     

    def getDrivetrainState(self, drivetrainState ): 
        self.driveTrainState = drivetrainState

    def getChassisSpeeds(self):
        return self.driveTrain.get_state().speeds    
    
    def getChassisSpeedsNorm(self):
        return hypot(self.driveTrain.get_state().speeds.vx,self.driveTrain.get_state().speeds.vy)    

    def getVx(self):
        return self.driveTrain.get_state().speeds.vx        
    
    def getVy(self):
        return self.driveTrain.get_state().speeds.vy            

    def getRotationalSpeedsRPS(self):
        return self.driveTrain.get_state().speeds.omega

    def getRotationalSpeedsDPS(self):
        return self.driveTrain.get_state().speeds.omega_dps
    
    def getPose(self):
        return self.driveTrain.get_state().pose
    
    def getX(self):
        return self.driveTrain.get_state().pose.translation().x        
    
    def getY(self):
        return self.driveTrain.get_state().pose.translation().y

    def getRotationDeg(self):
        return self.driveTrain.get_state().pose.rotation().degrees()            
    
    def getRotationRad(self):
        return self.driveTrain.get_state().pose.rotation().radians()                
                        
    
    def update(self):
        self.driveTrainState = self.driveTrain.get_state()
        self.displayState()



    def displayState(self):
        SmartDashboard.putNumber("Vx: ",self.getVx())
        SmartDashboard.putNumber("Vy: ",self.getVy())
        SmartDashboard.putNumber("Rot Rate: ",self.getRotationalSpeedsDPS())   
        SmartDashboard.putNumber("Rot Rate rps: ",self.getRotationalSpeedsRPS())           
        SmartDashboard.putNumber("X: ",self.getX())
        SmartDashboard.putNumber("y: ",self.getY())
        SmartDashboard.putNumber("Rot: ",self.getRotationDeg())     


    