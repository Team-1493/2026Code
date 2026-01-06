import math
from wpimath.geometry import Pose2d,Rotation2d,Translation2d
from wpimath.kinematics import ChassisSpeeds
from AutoPilot_py.Profile import APProfile
from AutoPilot_py.APTarget import ap_target
from dataclasses import dataclass, field
from math import hypot


@dataclass
class APResult:
    """Represents a Limelight Raw Fiducial result from Limelight's NetworkTables output."""
    vx: float = 0
    vy: float = 0
    targetAngle: Rotation2d = field(default_factory=Rotation2d)


""""
 Autopilot is a class that tries to drive a target to a goal in 2-D space.
 Autopilot is a stateless algorithm; as such, it does not "think ahead" and cannot avoid obstacles.
 Any math that Autopilot needs is already worked out such that only a small amount of computation is necessary on the fly. 
 Autopilot is designed to be used in a drivetrain's control loop, where the current state of the robot is passed in, and the next velocity is returned.
 """
class Autopilot:
  

    def __init__(self, profile: APProfile):
        """
        Constructs an Autopilot from a given profile. This is the profile that the autopilot will use for all actions.
        """
        self.m_profile = profile
        self.dt = 0.020


  

    def toTargetCoordinateFrame(self,coords: Translation2d ,target:  ap_target):
        """
        Turns any other coordinate frame into a coordinate frame with positive x meaning in the
        direction of the target's entry angle, if applicable (otherwise no change to angles).
        """
        if target.m_entryAngle is None:
            entryAngle = Rotation2d(0)
        else: entryAngle = target.m_entryAngle

        return coords.rotateBy(-entryAngle )



    def calculateMaxVelocity(self, dist: float,  endVelo: float):
        """ 
        Determines the maximum velocity required to travel the given distance and end at the desired
        end velocity.
        dist = The distance to travel, in meters
        endVelo = The desired end velocity, in m/s
        """
        return pow( (4.5 * pow(dist, 2.0)) * self.m_profile.constraints.jerk, (1.0 / 3.0)) + endVelo
  

    def correct(self,initial: Translation2d ,  goal: Translation2d):
        """"
        Attempts to drive the initial translation to the goal translation using the parameters for
        acceleration given in the profile. 
        initial =  The initial translation to drive from
        goal = The goal translation to drive to
        """
        angleOffset = Rotation2d()
        if (goal != Translation2d()) :
            angleOffset = Rotation2d(goal.X(), goal.Y())

        adjustedGoal = goal.rotateBy(-angleOffset)
        adjustedInitial = initial.rotateBy(-angleOffset)
        initialI = adjustedInitial.X()
        goalI = adjustedGoal.X()
        # we cap the adjusted I because we'd rather adjust now than overshoot.
        if (goalI > self.m_profile.constraints.velocity):
            goalI = self.m_profile.constraints.velocity
    
        adjustedI = min(goalI,
            self.push(initialI, goalI, self.m_profile.constraints.acceleration))
        
        return Translation2d(adjustedI, 0).rotateBy(angleOffset)
  

   
    def push(self,start: float,  end: float,  accel: float):
        """"
        Using the provided acceleration, "pushes" the start point towards the end point.
        This is used for ensuring that changes in velocity are withing the acceleration threshold.
        """
        maxChange = accel * self.dt
        if (abs(start - end) < maxChange): 
            return end
    
        if (start > end): 
            return start - maxChange
    
        return start + maxChange
  

    def toGlobalCoordinateFrame(self,coords: Translation2d, target: ap_target ): 
        """"
        Turns a translation from a target-relative coordinate frame to a global coordinate frame.
   
        """
        if target.m_entryAngle is None:
           entryAngle = Rotation2d(0)
        else:
           entryAngle = target.m_entryAngle
        return coords.rotateBy(entryAngle)


    def calculateSwirlyVelocity(self,offset: Translation2d , target:  ap_target):
        """
        Uses the swirly method to calculate the correct velocities for the robot, 
        respecting entry angles.

        offset = The offset from the robot to the target, in the target's coordinate frame
        target =  The target that Autopilot is trying to reach
        """
        disp = offset.norm()
        theta = Rotation2d(offset.X(), offset.Y())
        rads = theta.radians()
        dist = self.calculateSwirlyLength(rads, disp)
        vx = theta.cos() - rads * theta.sin()
        vy = rads * theta.cos() + theta.sin()
        # normalize and scale
        return (Translation2d(vx, vy)/hypot(vx, vy))*(self.calculateMaxVelocity(dist, target.m_velocity))
  

    def calculateSwirlyLength(self, theta: float,radius: float):
        """"
        Using a precomputed integral, returns the length of the path that the swirly method generates.
        More specifically, this calculates the arc length of the polar curve r=theta from the given
        angle to zero, then scales it to match the current state.

        theta =  The angle of the offset from the robot to the target, in radians
        radius =  The normalized offset from the robot to the target, in meters
        """
        if (theta == 0):
            return radius
    
        theta = abs(theta)
        hypotenuse = hypot(theta, 1)
        u1 = radius * hypotenuse
        u2 = radius * math.log(theta + hypotenuse) / theta
        return 0.5 * (u1 + u2)
  


    def getRotationTarget(self, current: Rotation2d ,target:  ap_target ,  dist: float):
        """
        Returns the target's rotation if the robot is within a specified rotation radius; otherwise, 
        returns the current rotation of the robot.
        current =  The current rotation of the robot.
        target =  The APTarget that Autopilot is trying to reach.
        dist =  The distance from the robot to the target.
        """
        if target.m_rotationRadius is None:
            return target.m_reference.rotation()
    
        radius = target.m_rotationRadius
        if (radius > dist):
            return target.m_reference.rotation()
        else:
            return current

    def atTarget(self,  current: Pose2d,  target: ap_target):
        """
        Return whether the given pose is within the tolerance of the APTarget.
        current =  The current pose of the robot.
        target =  The APTarget to check against.
        Returns true if Autopilot has reached the target.
        """
        goal = target.m_reference
        goal = target.m_reference
       
        xyErr =hypot(current.X() - goal.X(),current.Y() - goal.Y()) 
        okXY = xyErr <= self.m_profile.errorXY
       
        thetaErr = abs(current.rotation().radians() - (goal.rotation()).radians())
        if thetaErr>math.pi: thetaErr = thetaErr - 2*math.pi
        if thetaErr<-math.pi: thetaErr = thetaErr - 2*math.pi 
        okTheta = abs(thetaErr) <= self.m_profile.errorTheta

        print(round(goal.X(),3),round(goal.Y(),3),round(current.X(),3),
            round(current.Y(),3), round(xyErr,3),round(goal.rotation().radians(),3),
            round(current.rotation().radians(),3),round(thetaErr,3))
        return okXY and okTheta

        """"
        xErr = abs(current.X() - goal.X())
        yErr = abs(current.Y() - goal.Y())
        thetaErr = abs(current.rotation().radians() - (goal.rotation()).radians())
        if thetaErr>math.pi: thetaErr = thetaErr - 2*math.pi
        if thetaErr<-math.pi: thetaErr = thetaErr - 2*math.pi
 #       print(round(goal.X(),3),round(goal.Y(),3),round(current.X(),3),
 #        round(current.Y(),3), round(xyErr,3),round(thetaErr,3))
        okX = xErr <= self.m_profile.errorXY
        okY = yErr <= self.m_profile.errorXY        
        okTheta = thetaErr <= self.m_profile.errorTheta
        return okX and okY and okTheta
        """


    def calculate(self,current: Pose2d, robotRelativeSpeeds: ChassisSpeeds, target: ap_target):
        """
        Returns the next field relative velocity for the trajectory
        current =  The robot's current position.
        robotRelativeSpeeds = The robot's current <b>robot relative</b> ChassisSpeeds.
        target =  The target the robot should drive towards.
   
        returns an APResult containing the next velocity and target angle
        """
        offset = self.toTargetCoordinateFrame(
            target.m_reference.translation() - current.translation(), target)
        if offset == Translation2d() :
           return APResult(0,0,target.m_reference.rotation())
        
        fieldRelativeSpeeds = (
            Translation2d(robotRelativeSpeeds.vx,robotRelativeSpeeds.vy).
            rotateBy(current.rotation()))
        initial = self.toTargetCoordinateFrame(fieldRelativeSpeeds, target)
        disp = offset.norm()

        if ((target.m_entryAngle is None) or disp < self.m_profile.beelineRadius) :
            towardsTarget = offset/disp
            goal = towardsTarget * (self.calculateMaxVelocity(disp, target.m_velocity))

            out = self.correct(initial, goal)
            velo = self.toGlobalCoordinateFrame(out, target)
            rot = self.getRotationTarget(current.rotation(), target, disp)
            return APResult(velo.X(), velo.Y(), rot)
    
        goal = self.calculateSwirlyVelocity(offset, target)
        out = self.correct(initial, goal)
        velo = self.toGlobalCoordinateFrame(out, target)
        rot = self.getRotationTarget(current.rotation(), target, disp)
        return  APResult(velo.X(), velo.Y(), rot)
  


  