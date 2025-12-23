from math import pi
from robotpy_apriltag import  AprilTag,AprilTagField,AprilTagFieldLayout
from wpimath.geometry import Pose3d, Rotation3d,Translation3d

class ConstantValues():

    
    class LeftLimelightConstants():
        CAMERA_NAME = "limelightLeft"
        X_OFFSET = 0 # forward positive
        Y_OFFSET = 0 # right positive
        Z_OFFSET = 0 # up positive
        THETA_X_OFFSET = 0 # roll
        THETA_Y_OFFSET = 0 # pitch
        THETA_Z_OFFSET = 0 # yaw

    class RightLimelightConstants():
        CAMERA_NAME = "limelightRight"
        X_OFFSET = 0 # forward positive
        Y_OFFSET = 0 # right positive
        Z_OFFSET = 0 # up positive
        THETA_X_OFFSET = 0 # roll
        THETA_Y_OFFSET = 0 # pitch
        THETA_Z_OFFSET = 0 # yaw


    class VisionConstants():    
        tag1=AprilTag()
        tag1.ID = 1
        tag1.pose = Pose3d(Translation3d(2,	2, 0), Rotation3d(0.000, 0.000, pi))
        tag2=AprilTag()
        tag3=AprilTag()
        tag4=AprilTag()
        tag5=AprilTag()
        tag6=AprilTag()
        tag7=AprilTag()                        
        tag8=AprilTag()
        tag9=AprilTag()
        tag10=AprilTag()
        tag11=AprilTag()
        tag12=AprilTag()
        tag13=AprilTag()
        tag14=AprilTag()
        tag15=AprilTag()

        tag16=AprilTag()
        tag16.ID = 16
        tag16.pose = Pose3d(Translation3d(5.988,	-0.004,	1.3017), Rotation3d(0.000,	0.000, 1.571))

        tag17=AprilTag()
        tag17.ID = 17
        tag17.pose = Pose3d(Translation3d(4.074,	3.306,  0.308),  Rotation3d(0.000, 0.000, 4.189))
        
        tag18=AprilTag()                
        tag18.ID = 18
        tag18.pose = Pose3d(Translation3d(3.658,	4.026, 0.308),  Rotation3d(0.000, 0.000, 3.142))
        
        tag19=AprilTag()
        tag19.ID = 19
        tag19.pose = Pose3d(Translation3d(4.074,	4.745, 0.308),  Rotation3d(0.000, 0.000, 2.094))

        tag20=AprilTag()
        tag20.ID = 20
        tag20.pose = Pose3d(Translation3d( 4.905,	4.745, 0.308),  Rotation3d(0.000, 0.000, 1.047))

        tag21=AprilTag()        
        
        # tags facing robot have r3 = 180 degrees
        
#        tags_list = [tag1,tag2,tag3,tag4,tag5,tag6,tag7,tag8,tag9,tag10,tag11,
 #                    tag12,tag13,tag14,tag15,tag16,tag17,tag18,tag19,tag20,tag21]

        tags_list = [tag16,tag17,tag18,tag19,tag20]
        field_length_meters = 16.48 # Example field dimensions
        field_width_meters = 8.11 # Example field dimensions
        field_layout = AprilTagFieldLayout(tags_list, field_length_meters, field_width_meters)

#       
"""""
        AprilTag(3, Pose3d(11.561,	8.056, 1.302, Rotation3d(0.000, 0.000, 4.712))),
        AprilTag(4, Pose3d(9.276,    6.138,	1.868, Rotation3d(0.000,	0.524, 0.000))),
        AprilTag(5, Pose3d(9.276,    1.915,	1.868, Rotation3d(0.000,	0.524, 0.000))),
        AprilTag(6, Pose3d(13.474,	3.306, 0.308, Rotation3d(0.000, 0, 5.236))),
        AprilTag(7, Pose3d(13.891,	4.026, 0.308, Rotation3d(0.000, 0.000, 0.000))),
        AprilTag(8, Pose3d(13.474,	4.745, 0.308, Rotation3d(0.000, 0.000, 1.047))),
        AprilTag(9, Pose3d(12.643,	4.745, 0.308, Rotation3d(0.000, 0.000, 2.094))),
        AprilTag(10, Pose3d(12.227, 4.026, 0.308, Rotation3d(0.000, 0.000, 3.142))),
        AprilTag(11, Pose3d(12.643, 3.306, 0.308, Rotation3d(0.000, 0.000, 4.189))),
        AprilTag(12,  Pose3d(0.851, 0.655, 1.486,  Rotation3d(0.000, 0.000, 0.942))),
        AprilTag(13,  Pose3d(0.851, 7.396, 1.486, Rotation3d(0.000, 0, 5.341))),
        AprilTag(14,  Pose3d(8.272,	6.138, 1.868,  Rotation3d(0.000, 0.524, 3.142))),
        AprilTag(15,  Pose3d(8.272,	1.915, 1.868,  Rotation3d(0.000, 0.524, 3.142))),
        AprilTag(16,  Pose3d(5.988,	-0.004,	1.3017, Rotation3d(0.000,	0.000, 1.571))),
        AprilTag(21,  Pose3d(5.321,	4.026, 0.308,  Rotation3d(0.000, 0.000, 0.000))),
        AprilTag(22,  Pose3d(4.905,	3.306, 0.308,  Rotation3d(0.000, 0.000, 5.236)))
        """
        
    
        
        

