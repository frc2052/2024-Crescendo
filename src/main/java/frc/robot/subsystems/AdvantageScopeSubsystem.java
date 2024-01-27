package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
        Logger.RecordOutput("Under Bumper Intake Lower Motor Speed", 
        oneUnderBumperIntakeSubsystem.getLowerMotorSpeed());

        Logger.RecordOutput("Under Bumper Intake Upper Motor Speed", 
        oneUnderBumperIntakeSubsystem.getUpperMotorSpeed());

    }

    public static void recordOverBumberIntakeData () {
        Logger.RecordOutput("Is Over Bumber Intake In Solenoid Active", 
        overBumperIntakeSubsystem.isInSolonoidActive());

        Logger.RecordOutput("Is Over Bumber Intake Out Solenoid Active", 
        overBumperIntakeSubsystem.isOutSolonoidActive());
    }

    public static void recordVerticalShooterData () {
        Logger.RecordOutput("Vertical Shooter Upper Motor Speed", 
        verticalShooterSubsystem.getUpperShooterSpeed());
        
        Logger.RecordOutput("Vertical Shooter Lower Motor Speed", 
        verticalShooterSubsystem.getLowerShooterSpeed());
    }

    public static void recordClimberData () {
        Logger.RecordOutput("Climber Encoder Position", 
        climberSubsystem.getEncoderPosition());
    }

    public static void recordMusicPlayerData() {
        Logger.RecordOutput("Current Track Play Time",
        musicPlayerSubsystem.getCurrentPlayTime());

        Logger.RecordOutput("Is Music Player Playing", 
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
        Logger.RecordOutput("Swerve Array", swerveArray);
    }
}
