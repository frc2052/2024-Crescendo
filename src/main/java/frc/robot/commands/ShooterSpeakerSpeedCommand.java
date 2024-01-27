package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.VerticalShooterSubsystem;
import frc.robot.subsystems.shooter.VerticalShooterSubsystem.ShooterSpeeds;

public class ShooterSpeakerSpeedCommand extends Command{

    private final VerticalShooterSubsystem shooter;

    public ShooterSpeakerSpeedCommand(VerticalShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setSpeed(ShooterSpeeds.SPEAKER);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setSpeed(ShooterSpeeds.OFF);
    }
}