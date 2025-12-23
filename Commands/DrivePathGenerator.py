import typing
from commands2 import Command, Subsystem
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath, PathConstraints, GoalEndState,Waypoint,IdealStartingState
from wpilib import DriverStation
from wpimath.kinematics import ChassisSpeeds;
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from subsystems.Drive.driveTrainGenerate import DrivetrainGenerator
from Constants import ConstantValues 
#from wpimath.geometry import Pose2d
import math
from math import pi



class DrivePath():
    def __init__(self,
                _robot: typing.Callable[[], Pose2d],
                 ) -> None:
        super().__init__()

        self.driveTrain = DrivetrainGenerator.getInstance()
        self.robot = _robot
        self.targetPose= Pose2d(Translation2d(3,3),Rotation2d(1.57))
        self.robotPose=self.robot()
        self.running=False
        self.path = None

        self.constraints = PathConstraints(
            1.75, 
            2,
            pi,
            5*pi/4)



# Generate a commanmd to drive to a given pose using path-on-the-fly (create a path from the start and end poses)  
# Control the direction of travel at each waypiont usaing the the rotation of the poses used in wayppointsFromPoses
# Robot rotation controlled by GoalEndState
# Does not avoid feild elements
    def drivePathToPose(self,_targetPose, finalVel=None) -> Command:
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
        print("iss V: ",iss.velocity, "  iss R: ",iss.rotation)

        self.path =  PathPlannerPath(
            self.waypoints, 
            self.constraints,iss, 
            GoalEndState(finalVel, self.targetPose.rotation())
        )

        self.path.preventFlipping = True
        return ((AutoBuilder.followPath(self.path)))
    

# Generate a command to drive a path between two poses using PathFind.
# Will avoid field elements, but no control over direction of travel
    def drivePathFindToPose(self,_targetPose, finalVel=None) -> Command:
        self.targetPose=_targetPose
        self.robotPose=self.robot()
        if finalVel is None:
            finalVel = 0
        return(AutoBuilder.pathfindToPose(self.targetPose, self.constraints,finalVel))           


# Determine a goal pose relative to a given tag. 
    def calculatePoseGoalFromTag(self, i : int, x_offset = None, y_offset = None):
        poseTag = ConstantValues.VisionConstants.tags_list[i-1].pose.toPose2d()
        rotTag = poseTag.rotation().radians()
        transTag = poseTag.translation()
        rotRobot=rotTag+pi

        poseGoal =  Pose2d(
            transTag.X()-x_offset*math.sin(rotTag)-y_offset*math.cos(rotTag),
            transTag.Y()-x_offset*math.cos(rotTag)+(y_offset)*math.sin(rotTag),
            Rotation2d(rotRobot))

        return (poseGoal)



# Get a command to drive a path-on-the-fly from robot position to a tag.
# Provide april Tag numnber, optoinal offset perpendicular to tag, offset parallel to tag
    def drivePathToTag(self,i:int, x_offset = None, y_offset = None,velFinal = None):
        return self.drivePathToPose(self.calculatePoseGoalFromTag(i,x_offset,y_offset),
                                    velFinal)

# Get a command to pathfind from robot position to a tag.
# Provide april Tag numnber, optoinal offset perpendicular to tag, offset parallel to tag
    def drivePathFindToTag(self,i:int, x_offset = None, y_offset = None, velFinal = None):
        return self.drivePathFindToPose(self.calculatePoseGoalFromTag(i,x_offset,y_offset),
                                        velFinal)

      
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
