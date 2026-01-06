# all distances in meters, all angles in radians
#
# A class that holds constraint information for an Autopilot action.
# 
# Constraints are max velocity, acceleration, and jerk.
#

import math



class APConstraints:
    def __init__(self,vel :float = math.inf, acc : float | None = None, jerk: float | None = None):
        """ 
        velocity  - The maximum velocity that Autopilot will demand, in m/s
        acceleration -  The maximum acceleration that Autopilot action will use to correct initial velocities, in m/s^2
        jerk - The maximum jerk that Autopilot will use to decelerate at the end of an action, in m/s^3
        """
        self.velocity = vel
        self.acceleration = acc
        self.jerk = jerk



    def with_velocity(self, vel):
        """
        Modifies this APConstraints object's max velocity and returns itself. 
        This affects the maximum velocity that Autopilot can demand. 
        velocity  - The maximum velocity that Autopilot will demand, in m/s
        """
        self.velocity = vel
        return self
    



    def with_acceleration(self, acc):
        """
        Modifies this APConstraint object's acceleration and returns itself. This affects the maximum
        acceleration that Autopilot will use to correct initial velocities.
        Autopilot's acceleration is used at the beginning of an action (not relevant to Autopilot's end behavior). 
        acceleration - The maximum acceleration that Autopilot will use to start a path, in m/s^2
        """
        self.acceleration = acc
        return self
    

    def with_jerk(self, jk):
        """
        Modifies this constraint's max jerk value and returns itself. Higher values mean a faster
        deceleration. Autopilot's jerk is used at the end of an action (not relevant to Autopilot's start behavior). 
        jerk -  The maximum jerk that Autopilot will use to decelerate at the end of an action, in m/s^3
        """ 
        self.jerk = jk
        return self   

