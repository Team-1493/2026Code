from generated.tuner_constants import TunerConstants

class DrivetrainGenerator():
    instance=None

    @staticmethod
    def getInstance():
        if DrivetrainGenerator.instance == None:
            DrivetrainGenerator.instance = TunerConstants.create_drivetrain()
            print("**********************  CFREATING DT  **********************") 
        return DrivetrainGenerator.instance
    