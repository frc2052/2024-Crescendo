package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.VerticalShooterSubsystem;

public class VerticalShooterMidRangeShotCommands extends Command {
private VerticalShooterSubsystem shooter;

public VerticalShooterMidRangeShotCommands(VerticalShooterSubsystem shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
}

    @Override
    public void initialize () {
        shooter.setLowerShooterSpeed(0);
        shooter.setUpperShooterSpeed(0);
    }
}
