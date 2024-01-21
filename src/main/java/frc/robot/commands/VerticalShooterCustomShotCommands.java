package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.VerticalShooterSubsystem;

public class VerticalShooterCustomShotCommands extends Command {
private VerticalShooterSubsystem shooter;
private double newShooterTPSspeed;

public VerticalShooterCustomShotCommands(VerticalShooterSubsystem shooter, double newShooterTPSspeed) {
    this.shooter = shooter;
    this.newShooterTPSspeed = newShooterTPSspeed;
    addRequirements(shooter);
    }

    @Override
    public void initialize () {
        shooter.setLowerShooterSpeed(newShooterTPSspeed);
        shooter.setUpperShooterSpeed(newShooterTPSspeed);
    }
}
