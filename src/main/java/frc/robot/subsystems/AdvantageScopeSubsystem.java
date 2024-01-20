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

    public static void recordDrivetrainData () {

    }




}
