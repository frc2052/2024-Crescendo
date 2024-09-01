package frc.robot.commands.autonomous.backupAuto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoDriveCommand extends Command {
    private DrivetrainSubsystem drivetrain;
    private Timer driveTimer;
    private double time;
    private double xSpeed;
    private double ySpeed;
    
    public AutoDriveCommand (DrivetrainSubsystem drivetrain, double driveTime, double xSpeed, double ySpeed) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
        this.driveTimer = new Timer();
        time = driveTime;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
    }

    @Override
    public void initialize () {
        driveTimer.start();
        drivetrain.drive(xSpeed, ySpeed, 0, true);
    }

    @Override
    public void execute () {
        if (driveTimer.get() >= time) {
            drivetrain.stop();
        }
    }
}
