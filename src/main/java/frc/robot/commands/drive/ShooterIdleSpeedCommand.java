package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.VerticalShooterSubsystem;
import frc.robot.subsystems.shooter.VerticalShooterSubsystem.ShooterSpeeds;

public class ShooterIdleSpeedCommand extends Command{

    private final VerticalShooterSubsystem shooter;

    public ShooterIdleSpeedCommand(VerticalShooterSubsystem shooter) {
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
