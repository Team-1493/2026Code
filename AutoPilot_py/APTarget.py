from wpimath.geometry import Pose2d, Rotation2d




class ap_target:
    """
    The APTarget class represents the goal end state of an Autopilot action. 
    Target needs a reference Pose2d, but can optionally have a specified entry angle and rotation radius.

    A target may also specify an end velocity or end velocity.
    """        


    def __init__(self,pose:Pose2d):
        """ Creates a new Autopilot target with the given target pose, no entry angle, and no end velocity.
            pose =  The reference pose for this target.
        """
        self.m_reference = pose
        self.m_velocity = 0
        self.m_entryAngle = None
        self.m_rotationRadius = None
  

  
    def with_reference(self,ref: Pose2d ) :
        """
        Returns a copy of this target with the given reference Pose2d.
        """
        target = self.clone()
        target.m_reference = ref
        return target
  

    def with_entry_angle(self,angle: Rotation2d ) :
        """
        Returns a copy of this target with the given entry angle.
        """
        target = self.clone()
        target.m_entryAngle = angle
        return target


    def with_velocity(self, vel:float ) :
        """
        Returns a copy of this target with the given endf velocity.
        """
        target = self.clone()
        target.m_velocity= vel
        return target


    def with_rotation_radius(self,radius: float ) :
        """
        Returns a copy of this target with the given rotation radius.
        By default, rotation goals are always respected. Adjusting this radius prevents Autopilot from reorienting
        the robot until the robot is within the specified radius of the target.
        """
        target = self.clone()
        target.m_rotationRadius = radius
        return target


    def get_reference(self):
        """ Returns this target's reference Pose2d """
        return self.m_reference
  
    def get_entry_angle(self):
        """ Returns this target's desired entry angle """
        return self.m_entryAngle

    def get_velocity(self):
        """ Returns this target's end velocity """
        return self.m_velocity


    def get_rotation_radius(self):
        """ Returns this target's rotation radius """
        return self.m_rotationRadius


    def clone(self):
        """" Retuns a copy of this target """
        target = ap_target(self.m_reference)
        target.m_velocity = self.m_velocity
        target.m_entryAngle = self.m_entryAngle
        target.m_rotationRadius = self.m_rotationRadius
        return target



    def withoutEntryAngle(self):
        """ Retuns a copy of this target, without the entry angle set. 
            This is useful if trying to make two different targets with and without entry angle set.
        """        
        target = ap_target(self.m_reference)
        target.m_velocity = self.m_velocity
        target.m_rotationRadius =self.m_rotationRadius
        return target
  

