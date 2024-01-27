package frc.robot.subsystems;

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
    
    public AdvantageScopeSubsystem (    
    OneUnderBumperIntakeSubsystem oneUnderBumperIntakeSubsystem, 
    OverBumperIntakeSubsystem overBumperIntakeSubsystem, 
    VerticalShooterSubsystem verticalShooterSubsystem, 
    ClimberSubsystem climberSubsystem, 
    DrivetrainSubsystem drivetrainSubsystem) {

        AdvantageScopeSubsystem.oneUnderBumperIntakeSubsystem = oneUnderBumperIntakeSubsystem;
        AdvantageScopeSubsystem.overBumperIntakeSubsystem = overBumperIntakeSubsystem;
        AdvantageScopeSubsystem.verticalShooterSubsystem = verticalShooterSubsystem;
        AdvantageScopeSubsystem.climberSubsystem = climberSubsystem;
        AdvantageScopeSubsystem.drivetrainSubsystem = drivetrainSubsystem;
        
        recordOneUnderBumberIntakeData();

        recordOverBumberIntakeData();

        recordVerticalShooterData();

        recordClimberData();

        recordDrivetrainData();
    }    

    public static void recordOneUnderBumberIntakeData () {
        SmartDashboard.getNumber("Under Bumper Intake Lower Motor Speed", 
        oneUnderBumperIntakeSubsystem.getLowerMotorSpeed());

        SmartDashboard.getNumber("Under Bumper Intake Upper Motor Speed", 
        oneUnderBumperIntakeSubsystem.getUpperMotorSpeed());
    }

    public static void recordOverBumberIntakeData () {
        SmartDashboard.getBoolean("Is Over Bumber Intake In Solenoid Active", 
        overBumperIntakeSubsystem.isInSolonoidActive());

        SmartDashboard.getBoolean("Is Over Bumber Intake Out Solenoid Active", 
        overBumperIntakeSubsystem.isOutSolonoidActive());
    }

    public static void recordVerticalShooterData () {
        SmartDashboard.getNumber("Vertical Shooter Upper Motor Speed", 
        verticalShooterSubsystem.getUpperShooterSpeed());
        
        SmartDashboard.getNumber("Vertical Shooter Lower Motor Speed", 
        verticalShooterSubsystem.getLowerShooterSpeed());
    }

    public static void recordClimberData () {
        SmartDashboard.getNumber("Climber Encoder Position", 
        climberSubsystem.getEncoderPosition());
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
        SmartDashboard.getNumberArray("Swerve Array", swerveArray);
    }
}
