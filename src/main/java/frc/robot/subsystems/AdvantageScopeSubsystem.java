package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public class AdvantageScopeSubsystem extends SubsystemBase {
    static IntakeSubsystem intake;
    static ShooterSubsystem shooter;
    static ClimberSubsystem climber;
    static DrivetrainSubsystem drivetrain;
    static MusicPlayerSubsystem musicPlayer;
    
    public AdvantageScopeSubsystem (    
        IntakeSubsystem intake,
        ShooterSubsystem shooter, 
        ClimberSubsystem climber, 
        DrivetrainSubsystem drivetrain,
        MusicPlayerSubsystem musicPlayer
    ) {

        AdvantageScopeSubsystem.intake = intake;
        AdvantageScopeSubsystem.shooter = shooter;
        AdvantageScopeSubsystem.climber = climber;
        AdvantageScopeSubsystem.drivetrain = drivetrain;
        AdvantageScopeSubsystem.musicPlayer = musicPlayer;
    }    

    public void recordData() {
        recordIntakeData();

        recordShooterData();

        recordClimberData();

        recordDrivetrainData();

        recordMusicPlayerData();
    }

    public static void recordIntakeData () {
        Logger.recordOutput("m_Under Bumper Intake Lower Motor Speed", 
        intake.getLowerMotorSpeed());

        Logger.recordOutput("m_Under Bumper Intake Upper Motor Speed", 
        intake.getUpperMotorSpeed());

    }

    public static void recordShooterData () {
        Logger.recordOutput("m_Vertical Shooter Upper Motor Speed", 
        shooter.getUpperShooterSpeed());
        
        Logger.recordOutput("m_Vertical Shooter Lower Motor Speed", 
        shooter.getLowerShooterSpeed());
    }

    public static void recordClimberData () {
        Logger.recordOutput("m_Climber Encoder Position", 
        climber.getEncoderPosition());
    }

    public static void recordMusicPlayerData() {
        Logger.recordOutput("m_Current Track Play Time",
        musicPlayer.getCurrentPlayTime());

        Logger.recordOutput("m_Is Music Player Playing", 
        musicPlayer.isPlayerPLaying());
    }

    public static void recordDrivetrainData () {
        double[] swerveArray = {
            drivetrain.frontLeftModule.getState().angle.getDegrees(), 
            drivetrain.frontLeftModule.getState().speedMetersPerSecond,

            drivetrain.frontRightModule.getState().angle.getDegrees(), 
            drivetrain.frontRightModule.getState().speedMetersPerSecond,

            drivetrain.backLeftModule.getState().angle.getDegrees(), 
            drivetrain.backLeftModule.getState().speedMetersPerSecond,

            drivetrain.backRightModule.getState().angle.getDegrees(), 
            drivetrain.backRightModule.getState().speedMetersPerSecond,
        };
        Logger.recordOutput("m_Swerve Array", swerveArray);

        Logger.recordOutput("m_Pose 2D", RobotState.getInstance().getRobotPose());

        //SmartDashboard.getNumber("");
    }
}