package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.VerticalShooterSubsystem;
import frc.robot.subsystems.shooter.VerticalShooterSubsystem.ShooterSpeeds;

public class ShooterAmpSpeedCommand extends Command{

    private final VerticalShooterSubsystem shooter;

    public ShooterAmpSpeedCommand(VerticalShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setSpeed(ShooterSpeeds.IDLING);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setSpeed(ShooterSpeeds.OFF);
    }
}
