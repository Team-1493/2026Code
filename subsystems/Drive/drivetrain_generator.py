from generated.tuner_constants import TunerConstants
from phoenix6.hardware import TalonFX,TalonFXS
from phoenix6 import configs
from subsystems.Drive.command_swerve_drivetrain import CommandSwerveDrivetrain
from Constants1 import ConstantValues
from phoenix6.configs import Slot0Configs

class DrivetrainGenerator():
    instance : CommandSwerveDrivetrain
    instance = None
    slot0_auto = Slot0Configs()
    slot0_teleop = Slot0Configs()


    @staticmethod
    def getInstance():
        if DrivetrainGenerator.instance == None:
            DrivetrainGenerator.instance = TunerConstants.create_drivetrain()
            print("**********************  CFREATING DT  **********************") 
        return DrivetrainGenerator.instance


    @staticmethod
    def updateGains():
    

        k_p_tele = ConstantValues.DriveConstants.TELEOP_kP
        k_v_tele = ConstantValues.DriveConstants.TELEOP_kV
        k_s_tele = ConstantValues.DriveConstants.TELEOP_kS
        k_a_tele = ConstantValues.DriveConstants.TELEOP_kA
        
        DrivetrainGenerator.slot0_teleop.k_p = k_p_tele
        DrivetrainGenerator.slot0_teleop.k_s = k_s_tele
        DrivetrainGenerator.slot0_teleop.k_v = k_v_tele
        DrivetrainGenerator.slot0_teleop.k_a = k_a_tele                        

        k_p_auto = ConstantValues.DriveConstants.AUTO_kP
        k_v_auto = ConstantValues.DriveConstants.AUTO_kV
        k_s_auto = ConstantValues.DriveConstants.AUTO_kS
        k_a_auto = ConstantValues.DriveConstants.AUTO_kA

        DrivetrainGenerator.slot0_auto.k_p = k_p_auto
        DrivetrainGenerator.slot0_auto.k_s = k_s_auto
        DrivetrainGenerator.slot0_auto.k_v = k_v_auto
        DrivetrainGenerator.slot0_auto.k_a = k_a_auto                                



    @staticmethod
    def apply_teleop_gains():
        DrivetrainGenerator.instance.get_module(0).drive_motor.configurator.apply(DrivetrainGenerator.slot0_teleop)
        DrivetrainGenerator.instance.get_module(1).drive_motor.configurator.apply(DrivetrainGenerator.slot0_teleop)
        DrivetrainGenerator.instance.get_module(2).drive_motor.configurator.apply(DrivetrainGenerator.slot0_teleop)
        DrivetrainGenerator.instance.get_module(3).drive_motor.configurator.apply(DrivetrainGenerator.slot0_teleop)
        
        
    @staticmethod
    def apply_auto_gains():
        DrivetrainGenerator.instance.get_module(0).drive_motor.configurator.apply(DrivetrainGenerator.slot0_auto)
        DrivetrainGenerator.instance.get_module(1).drive_motor.configurator.apply(DrivetrainGenerator.slot0_auto)
        DrivetrainGenerator.instance.get_module(2).drive_motor.configurator.apply(DrivetrainGenerator.slot0_auto)
        DrivetrainGenerator.instance.get_module(3).drive_motor.configurator.apply(DrivetrainGenerator.slot0_auto)
