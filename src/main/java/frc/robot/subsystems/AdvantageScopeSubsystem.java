package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.intake.OneUnderBumperIntakeSubsystem;
import frc.robot.subsystems.intake.OverBumperIntakeSubsystem;
import frc.robot.subsystems.shooter.VerticalShooterSubsystem;

public class AdvantageScopeSubsystem extends SubsystemBase {
    static OneUnderBumperIntakeSubsystem oneUnderBumperIntakeSubsystem;
    static OverBumperIntakeSubsystem overBumperIntakeSubsystem;
    static VerticalShooterSubsystem verticalShooterSubsystem;
    static ClimberSubsystem climberSubsystem;
    static DrivetrainSubsystem drivetrainSubsystem;
    static MusicPlayerSubsystem musicPlayerSubsystem;
    
    public AdvantageScopeSubsystem (    
    OneUnderBumperIntakeSubsystem oneUnderBumperIntakeSubsystem, 
    OverBumperIntakeSubsystem overBumperIntakeSubsystem, 
    VerticalShooterSubsystem verticalShooterSubsystem, 
    ClimberSubsystem climberSubsystem, 
    DrivetrainSubsystem drivetrainSubsystem,
    MusicPlayerSubsystem musicPlayerSubsystem) {

        AdvantageScopeSubsystem.oneUnderBumperIntakeSubsystem = oneUnderBumperIntakeSubsystem;
        AdvantageScopeSubsystem.overBumperIntakeSubsystem = overBumperIntakeSubsystem;
        AdvantageScopeSubsystem.verticalShooterSubsystem = verticalShooterSubsystem;
        AdvantageScopeSubsystem.climberSubsystem = climberSubsystem;
        AdvantageScopeSubsystem.drivetrainSubsystem = drivetrainSubsystem;
        AdvantageScopeSubsystem.musicPlayerSubsystem = musicPlayerSubsystem;
      
        recordOneUnderBumberIntakeData();

        recordOverBumberIntakeData();

        recordVerticalShooterData();

        recordClimberData();

        recordDrivetrainData();

        recordMusicPlayerData();
    }    

    public static void recordOneUnderBumberIntakeData () {
        Logger.recordOutput("Under Bumper Intake Lower Motor Speed", 
        oneUnderBumperIntakeSubsystem.getLowerMotorSpeed());

        Logger.recordOutput("Under Bumper Intake Upper Motor Speed", 
        oneUnderBumperIntakeSubsystem.getUpperMotorSpeed());

    }

    public static void recordOverBumberIntakeData () {
        Logger.recordOutput("Is Over Bumber Intake In Solenoid Active", 
        overBumperIntakeSubsystem.isInSolonoidActive());

        Logger.recordOutput("Is Over Bumber Intake Out Solenoid Active", 
        overBumperIntakeSubsystem.isOutSolonoidActive());
    }

    public static void recordVerticalShooterData () {
        Logger.recordOutput("Vertical Shooter Upper Motor Speed", 
        verticalShooterSubsystem.getUpperShooterSpeed());
        
        Logger.recordOutput("Vertical Shooter Lower Motor Speed", 
        verticalShooterSubsystem.getLowerShooterSpeed());
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