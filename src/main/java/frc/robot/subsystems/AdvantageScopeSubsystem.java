package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public class AdvantageScopeSubsystem extends SubsystemBase {
    static IntakeSubsystem intakeSubsystem;
    static ShamperSubsystem shamperSubsystem;
    static ClimberSubsystem climberSubsystem;
    static DrivetrainSubsystem drivetrainSubsystem;
    static MusicPlayerSubsystem musicPlayerSubsystem;
    //static VisionSubsystem visionSubsystem;
    static IndexerSubsystem indexerSubsystem;
    //static TrapArmSubsystem trapArmSubsystem;
    static String folder = "Data_";
    
    public AdvantageScopeSubsystem (    
    IntakeSubsystem intakeSubsystem, 
    ShamperSubsystem shamperSubsystem, 
    ClimberSubsystem climberSubsystem, 
    DrivetrainSubsystem drivetrainSubsystem,
    MusicPlayerSubsystem musicPlayerSubsystem,
    //VisionSubsystem visionSubsystem,
    IndexerSubsystem indexerSubsystem
    //,TrapArmSubsystem trapArmSubsystem
    ) {

        AdvantageScopeSubsystem.intakeSubsystem = intakeSubsystem;
        AdvantageScopeSubsystem.shamperSubsystem = shamperSubsystem;
        AdvantageScopeSubsystem.climberSubsystem = climberSubsystem;
        AdvantageScopeSubsystem.drivetrainSubsystem = drivetrainSubsystem;
        AdvantageScopeSubsystem.musicPlayerSubsystem = musicPlayerSubsystem;
        //AdvantageScopeSubsystem.visionSubsystem = visionSubsystem;
        AdvantageScopeSubsystem.indexerSubsystem = indexerSubsystem;
        //AdvantageScopeSubsystem.trapArmSubsystem = trapArmSubsystem;
    }

    public static void startRecording() {
        recordIntakeData();
        recordShamperData();
        recordClimberData();
        recordDrivetrainData();
        recordMusicPlayerData();
        //recordVisionData();
        recordIndexerData();
        //recordTrapArmData();
    }

    public static void recordIntakeData () {
        String intakeFolder = "Intake_";

        Logger.recordOutput(folder+intakeFolder+"Intake Speed", 
        intakeSubsystem.getIntakeMotorSpeed());

    }

    public static void recordIndexerData() {}

    // public static void recordTrapArmData() {
    //     String trapArmFolder = "TrapArm_";

    //     Logger.recordOutput(folder+trapArmFolder+"Trap Arm Position", 
    //     trapArmSubsystem.getPosition());
    // }

    public static void recordShamperData () {
        String shamperFolder = "Shamper_";

        Logger.recordOutput(folder+shamperFolder+"Shamper Upper Motor Speed", 
        shamperSubsystem.getUpperShamperSpeed());
        
        Logger.recordOutput(folder+shamperFolder+"Shamper Lower Motor Speed", 
        shamperSubsystem.getLowerShamperSpeed());
    }

    public static void recordClimberData () {
        String climberFolder = "Climber_";

        Logger.recordOutput(folder+climberFolder+"Climber Encoder Position", 
        climberSubsystem.getEncoderPosition());
    }

    public static void recordMusicPlayerData() {
        String musicPlayerFolder = "MusicPlayer_";

        Logger.recordOutput(folder+musicPlayerFolder+"Current Track Play Time",
        musicPlayerSubsystem.getCurrentPlayTime());

        Logger.recordOutput(folder+musicPlayerFolder+"Is Music Player Playing", 
        musicPlayerSubsystem.isPlayerPLaying());
    }

    public static void recordDrivetrainData () {
        String drivetrainFolder = "Drivetrain_";

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
        Logger.recordOutput(folder+drivetrainFolder+"Swerve Array", swerveArray);

        Logger.recordOutput(folder+drivetrainFolder+"Pose 2D", RobotState.getInstance().getRobotPose());
    }

    // public static void recordVisionData() {
    //     String visionFolder = "Vision_";

    //     Logger.recordOutput(folder+visionFolder+"Vision Has Target", visionSubsystem.hasTarget());

    //     Logger.recordOutput(folder+visionFolder+"Best Note Pose2D", visionSubsystem.getBestNotePose());
    // }

    public static void recordDriverStationData() {
        String driverstationFolder = "Driver Station_";

        Logger.recordOutput(folder+driverstationFolder+"Event Name", DriverStation.getEventName());

        Logger.recordOutput(folder+driverstationFolder+"Game Specific Message", DriverStation.getGameSpecificMessage());

        Logger.recordOutput(folder+driverstationFolder+"Match Number", DriverStation.getMatchNumber());

        Logger.recordOutput(folder+driverstationFolder+"Match Time", DriverStation.getMatchTime());

        Logger.recordOutput(folder+driverstationFolder+"Is Robot Enabled", DriverStation.isEnabled());
    }
}
