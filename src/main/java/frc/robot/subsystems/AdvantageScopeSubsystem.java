package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public class AdvantageScopeSubsystem extends SubsystemBase {
    static IntakeSubsystem intakeSubsystem;
    static ShooterSubsystem shooterSubsystem;
    static ClimberSubsystem climberSubsystem;
    static DrivetrainSubsystem drivetrainSubsystem;
    static MusicPlayerSubsystem musicPlayerSubsystem;
    
    public AdvantageScopeSubsystem (    
    IntakeSubsystem intakeSubsystem, 
    ShooterSubsystem shooterSubsystem, 
    ClimberSubsystem climberSubsystem, 
    DrivetrainSubsystem drivetrainSubsystem,
    MusicPlayerSubsystem musicPlayerSubsystem) {

        AdvantageScopeSubsystem.intakeSubsystem = intakeSubsystem;
        AdvantageScopeSubsystem.shooterSubsystem = shooterSubsystem;
        AdvantageScopeSubsystem.climberSubsystem = climberSubsystem;
        AdvantageScopeSubsystem.drivetrainSubsystem = drivetrainSubsystem;
        AdvantageScopeSubsystem.musicPlayerSubsystem = musicPlayerSubsystem;
      
        recordOneUnderBumberIntakeData();

        recordShooterData();

        recordClimberData();

        recordDrivetrainData();

        recordMusicPlayerData();
    }    

    public static void recordOneUnderBumberIntakeData () {
        Logger.recordOutput("Under Bumper Intake Lower Motor Speed", 
        intakeSubsystem.getLowerMotorSpeed());

        Logger.recordOutput("Under Bumper Intake Upper Motor Speed", 
        intakeSubsystem.getUpperMotorSpeed());

    }

    public static void recordShooterData () {
        Logger.recordOutput("Shooter Upper Motor Speed", 
        shooterSubsystem.getUpperShooterSpeed());
        
        Logger.recordOutput("Shooter Lower Motor Speed", 
        shooterSubsystem.getLowerShooterSpeed());
    }

    public static void recordClimberData () {
        Logger.recordOutput("Climber Encoder Position", 
        climberSubsystem.getEncoderPosition());
    }

    public static void recordMusicPlayerData() {
        Logger.recordOutput("Current Track Play Time",
        musicPlayerSubsystem.getCurrentPlayTime());

        Logger.recordOutput("Is Music Player Playing", 
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
        Logger.recordOutput("Swerve Array", swerveArray);

        Logger.recordOutput("Pose 2D", RobotState.getInstance().getRobotPose());
    }
}
