package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.VerticalShooterSubsystem;

public class VerticalShooterCustomShotCommands extends Command {
private VerticalShooterSubsystem shooter;

public VerticalShooterCustomShotCommands(VerticalShooterSubsystem shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
}

    @Override
    public void initialize () {
        shooter.setLowerShooterSpeed(0);
        shooter.setUpperShooterSpeed(0);
    }
}
