package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class AdvantageScopeSubsystem extends SubsystemBase {
    
    public AdvantageScopeSubsystem () {
        recordOneUnderBumberIntakeData();

        recordOverBumberIntakeData();

        recordFlatShooterData();

        recordHorizontalShooterData();

        recordVerticalShooterData();

        recordClimberData();

        recordDrivetrainData();

        recordMusicPlayerData();
    }    

    public static void recordOneUnderBumberIntakeData () {
        SmartDashboard.getNumber("Under Bumper Intake Lower Motor Speed", 
        RobotContainer.oneUnderBumperIntakeSubsystem.getLowerMotorSpeed());

        SmartDashboard.getNumber("Under Bumper Intake Upper Motor Speed", 
        RobotContainer.oneUnderBumperIntakeSubsystem.getUpperMotorSpeed());
    }

    public static void recordOverBumberIntakeData () {
        SmartDashboard.getBoolean("Is Over Bumber Intake In Solenoid Active", 
        RobotContainer.overBumperIntakeSubsystem.isInSolonoidActive());

        SmartDashboard.getBoolean("Is Over Bumber Intake Out Solenoid Active", 
        RobotContainer.overBumperIntakeSubsystem.isOutSolonoidActive());
    }

    public static void recordFlatShooterData () {
        SmartDashboard.getNumber("Flat Shooter Motor Speed", 
        RobotContainer.flatShooterSubsystem.getSpeed());
    }

    public static void recordHorizontalShooterData () {
        SmartDashboard.getNumber("Horizontal Shooter Left Motor Speed", 
        RobotContainer.horizontalShooterSubsystem.getLeftShooterSpeed());

        SmartDashboard.getNumber("Horizontal Shooter Right Motor Speed", 
        RobotContainer.horizontalShooterSubsystem.getRightShooterSpeed());
    }

    public static void recordVerticalShooterData () {
        SmartDashboard.getNumber("Vertical Shooter Upper Motor Speed", 
        RobotContainer.verticalShooterSubsystem.getUpperShooterSpeed());
        
        SmartDashboard.getNumber("Vertical Shooter Lower Motor Speed", 
        RobotContainer.verticalShooterSubsystem.getLowerShooterSpeed());
    }

    public static void recordClimberData () {
        SmartDashboard.getNumber("Climber Encoder Position", 
        RobotContainer.climberSubsystem.getEncoderPosition());
    }

    public static void recordMusicPlayerData() {
        SmartDashboard.getNumber("Current Track Play Time",
        RobotContainer.musicPlayerSubsystem.getCurrentPlayTime());

        SmartDashboard.getBoolean("Is Music Player Playing", 
        RobotContainer.musicPlayerSubsystem.isPlayerPLaying());
    }

    public static void recordDrivetrainData () {

        double[] swerveArray = {
            RobotContainer.drivetrain.frontLeftModule.getState().angle.getDegrees(), 
            RobotContainer.drivetrain.frontLeftModule.getState().speedMetersPerSecond,

            RobotContainer.drivetrain.frontRightModule.getState().angle.getDegrees(), 
            RobotContainer.drivetrain.frontRightModule.getState().speedMetersPerSecond,

            RobotContainer.drivetrain.backLeftModule.getState().angle.getDegrees(), 
            RobotContainer.drivetrain.backLeftModule.getState().speedMetersPerSecond,

            RobotContainer.drivetrain.backRightModule.getState().angle.getDegrees(), 
            RobotContainer.drivetrain.backRightModule.getState().speedMetersPerSecond,
        };

        SmartDashboard.getNumberArray("Swerve Array", swerveArray);
    }




}
