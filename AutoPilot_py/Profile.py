# all distances in meters, all angles in radians

from wpimath.units import meters 

from AutoPilot_py.Contraints import APConstraints

#  A class representing a profile that determines how Autopilot approaches a target.
#
#  The constraints property of the profile limits the robot's behavior.

#  Acceptable error for the controller (both translational and rotational) are stored here.
#  The "beeline radius" determines the distance at which the robot drives directly at the target and
#  no longer respects entry angle. This is helpful because if the robot overshoots by a small
#  amount, that error should not cause the robot do completely circle back around.

class APProfile:

  def __init__(self, const: APConstraints):
    """"
    Builds an APProfile with the given constraints. Tolerated error and beeline radius are all setto zero.
    constraints - The motion constraints for this profile
    """
    self.constraints = const
    self.errorXY = 0
    self.errorTheta = 0
    self.beelineRadius = 0

  def with_ErrorXY(self, eXY):
    """ Modifies this profile's tolerated xy error and returns itself"""
    self.errorXY = eXY
    return self
  
  def with_ErrorTheta(self, eTheta):
    """ Modifies this profile's tolerated angular error and returns itself"""
    self.errorTheta = eTheta
    return self 

  def with_Constraints(self, const: APConstraints):
    """ Modifies this profile's constraint and returns itslef"""    
    self.constraints = const
    return self



  def with_BeelineRadius(self, blRadius):
    """ 
    Modifies this profile's beeline radius and returns itself
    The beeline radius is a distance where, under that range, entry angle is no longer respected.
    This prevents small overshoots from causing the robot to make a full arc and instead correct
    itself. The distance at which the robot will drive directly at the target
    """    
    self.beelineRadius = blRadius
    a = APConstraints()
    return self


  
  def getErrorXY(self) :
    """"
    Returns the tolerated translation error for this profile.
    """    
    return self.errorXY
  


  def getErrorTheta(self) :
    """
    Returns the tolerated angular error for this profile.
    """
    return self.errorTheta
  


  def getConstraints(self) :
    """
    Returns the path generation constraints for this profile.
    """
    return self.constraints



  def getBeelineRadius(self) :
    """"
    Returns the beeline radius for this profile.
    """
    return self.beelineRadius