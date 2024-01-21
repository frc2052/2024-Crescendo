package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.VerticalShooterSubsystem;

public class VerticalShooterDefaultShotCommand extends Command {
    private VerticalShooterSubsystem shooter;

public VerticalShooterDefaultShotCommand(VerticalShooterSubsystem shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
}

    @Override
    public void initialize () {
        shooter.returnToIdleSpeed();
        shooter.returnToIdleSpeed();
    } 
}
