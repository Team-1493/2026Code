import typing
from commands2 import Command
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath, PathConstraints, GoalEndState,Waypoint,IdealStartingState,RotationTarget
from wpimath.kinematics import ChassisSpeeds;
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from subsystems.Drive.drivetrain_generator import DrivetrainGenerator
from subsystems.Drive.heading_controller import HeadingController
from Constants1 import ConstantValues 
import math
from math import pi
from Utilities.helper_methods import HelperMethods

from Commands.goal_pid import GoalPID
from Commands.goal_cam_command import GoalCamCommand



class DrivePathGenerator():
    def __init__(self,
                _robot: typing.Callable[[], Pose2d],
                 ) -> None:
        super().__init__()

        self.driveTrain = DrivetrainGenerator.getInstance()
        self.robot = _robot
        self.constantsVision = ConstantValues.getInstance().VisionConstants
        self.targetPose= Pose2d(Translation2d(3,3),Rotation2d(1.57))
        self.robotPose=self.robot()
        self.running=False
        self.path = None
        self.headingController = HeadingController.getInstance()
        self.constraints = PathConstraints(
            2.5, 
            3,
            2*pi,
            4*pi)


       

# Generate a commanmd to drive to a given pose using path-on-the-fly (create a path from the start and end poses)  
# Control the direction of travel at each waypiont usaing the the rotation of the poses used in wayppointsFromPoses
# Robot rotation controlled by GoalEndState
# Does not avoid feild elements
    def drive_path_to_pose(self,_targetPose: Pose2d, finalVel=None) -> Command:
        self.targetPose=_targetPose
        self.robotPose=self.robot()
        if finalVel is None:
            finalVel = 0

#             self.getPathVelocityHeading(self.getFieldVelocity(), self.targetPose))

        listOfPoses = [Pose2d(self.robotPose.translation(),
                   self.getPathVelocityHeading(self.getFieldVelocity())), self.targetPose]
        self.waypoints = PathPlannerPath.waypointsFromPoses(listOfPoses) 
            
        iss =  IdealStartingState(
        self.getVelocityMagnitude(self.getFieldVelocity()), self.robotPose.rotation())
        rotTarget=[]
        # complete the rotatation at the halfway point so all rotation is done
        # well ahead of handover to the next stage
        rotTarget.append(RotationTarget(0.5,self.targetPose.rotation()))
        self.path =  PathPlannerPath(
            self.waypoints, 
            self.constraints,iss, 
            GoalEndState(finalVel, self.targetPose.rotation()),
            rotTarget
        )

        self.path.preventFlipping = True
        return ((AutoBuilder.followPath(self.path)))
    

# Generate a command to drive a path between two poses using PathFind.
# Will avoid field elements, but no control over direction of travel
    def drive_pathfind_to_pose(self,_targetPose, finalVel=None) -> Command:
        self.targetPose=_targetPose
        self.robotPose=self.robot()
        if finalVel is None:
            finalVel = 0
        return(AutoBuilder.pathfindToPose(self.targetPose, self.constraints,finalVel))           


    def getPathVelocityHeading(self,cs: ChassisSpeeds):
        if self.getVelocityMagnitude(cs) < 0.25:
            diff = Translation2d(self.targetPose.translation().X()-self.robotPose.translation().X(),
                              self.targetPose.translation().Y()-self.robotPose.translation().Y())                                      
            if diff.norm() < 0.01:
                return self.targetPose.rotation()
            else:
                return diff.angle()        
        else: return Rotation2d(cs.vx, cs.vy)
    

    def getVelocityMagnitude(self, cs:ChassisSpeeds):
        self.driveTrain.get_state().speeds
        return Translation2d(cs.vx,cs.vy).norm()
    

    """
        // ChassisSpeeds has a method to convert from field-relative to robot-relative speeds,
        // but not the reverse.  However, because this transform is a simple rotation, negating the
        // angle given as the robot angle reverses the direction of rotation, and the conversion is
        // reversed.
    """

    def getFieldVelocity(self):
        robotRelativeSpeeds = self.driveTrain.kinematics.toChassisSpeeds(
            self.driveTrain.get_state().module_states)
        return ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, 
                                                    self.robotPose.rotation())


# Get a command to drive a path-on-the-fly from robot position to a tag.
# Provide april Tag numnber, optoinal offset perpendicular to tag, offset parallel to tag

    def drive_path_to_tag(self,i:int, x_offset = 0, y_offset = 0):
        goalPose1 = HelperMethods.calculate_pose_goal_from_tag(i,x_offset-.5,y_offset) 
        goalPose = HelperMethods.calculate_pose_goal_from_tag(i,x_offset,y_offset)

        return (
            (self.drive_path_to_pose(goalPose1,1))  #speed of 1m/s at transition
            .andThen(GoalCamCommand(x_offset,y_offset,i))
            .finallyDo(self.headingController.setTargetRotationInt))    
        
            
            

# Get a command to pathfind from robot position to a tag.
# Provide april Tag numnber, optoinal offset perpendicular to tag, offset parallel to tag
    def drive_pathfind_to_tag(self,i:int, x_offset = None, y_offset = None, velFinal = None):
        return self.drive_pathfind_to_pose(HelperMethods.calculate_pose_goal_from_tag(i,x_offset,y_offset),
                                        velFinal)

      
