package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.states.RobotState;

public class AdvantageScopeSubsystem extends SubsystemBase {
    static IntakeSubsystem intakeSubsystem;
    static ShamperSubsystem shamperSubsystem;
    static ClimberSubsystem climberSubsystem;
    static DrivetrainSubsystem drivetrainSubsystem;
    static MusicPlayerSubsystem musicPlayerSubsystem;
    static VisionSubsystem visionSubsystem;
    static String folder = "Data_";
    
    public AdvantageScopeSubsystem (    
    IntakeSubsystem intakeSubsystem, 
    ShamperSubsystem shamperSubsystem, 
    ClimberSubsystem climberSubsystem, 
    DrivetrainSubsystem drivetrainSubsystem,
    MusicPlayerSubsystem musicPlayerSubsystem,
    VisionSubsystem visionSubsystem) {

        AdvantageScopeSubsystem.intakeSubsystem = intakeSubsystem;
        AdvantageScopeSubsystem.shamperSubsystem = shamperSubsystem;
        AdvantageScopeSubsystem.climberSubsystem = climberSubsystem;
        AdvantageScopeSubsystem.drivetrainSubsystem = drivetrainSubsystem;
        AdvantageScopeSubsystem.musicPlayerSubsystem = musicPlayerSubsystem;
        AdvantageScopeSubsystem.visionSubsystem = visionSubsystem;
      
        recordOneUnderBumberIntakeData();

        recordShamperData();

        recordClimberData();

        recordDrivetrainData();

        recordMusicPlayerData();

        recordVisionData();
    }    

    public static void recordOneUnderBumberIntakeData () {
        Logger.recordOutput(folder+"Under Bumper Intake Lower Motor Speed", 
        intakeSubsystem.getLowerMotorSpeed());

        Logger.recordOutput(folder+"Under Bumper Intake Upper Motor Speed", 
        intakeSubsystem.getUpperMotorSpeed());

    }

    public static void recordShamperData () {
        Logger.recordOutput(folder+"Shamper Upper Motor Speed", 
        shamperSubsystem.getUpperShamperSpeed());
        
        Logger.recordOutput(folder+"Shamper Lower Motor Speed", 
        shamperSubsystem.getLowerShamperSpeed());
    }

    public static void recordClimberData () {
        Logger.recordOutput(folder+"Climber Encoder Position", 
        climberSubsystem.getEncoderPosition());
    }

    public static void recordMusicPlayerData() {
        Logger.recordOutput(folder+"Current Track Play Time",
        musicPlayerSubsystem.getCurrentPlayTime());

        Logger.recordOutput(folder+"Is Music Player Playing", 
        musicPlayerSubsystem.isPlayerPLaying());
    }

    public static void recordDrivetrainData () {
        double[] swerveArray = {
            drivetrainSubsystem.frontLeftModule.getState().angle.getDegrees(), 
            drivetrainSubsystem.frontLeftModule.getState().speedMetersPerSecond,

            drivetrainSubsystem.frontRightModule.getState().angle.getDegrees(), 
            drivetrainSubsystem.frontRightModule.getState().speedMetersPerSecond,

            drivetrainSubsystem.backLeftModule.getState().angle.getDegrees(), 
            drivetrainSubsystem.backLeftModule.getState().speedMetersPerSecond,

            drivetrainSubsystem.backRightModule.getState().angle.getDegrees(), 
            drivetrainSubsystem.backRightModule.getState().speedMetersPerSecond,
        };
        Logger.recordOutput(folder+"Swerve Array", swerveArray);

        Logger.recordOutput(folder+"Pose 2D", RobotState.getInstance().getRobotPose());

        //SmartDashboard.getNumber("");
    }

    public static void recordVisionData() {
        Logger.recordOutput(folder+"Visions Has Target", visionSubsystem.hasTarget());

        Logger.recordOutput(folder+"Best Note Pose2D", visionSubsystem.getBestNotePose());
    }
}
