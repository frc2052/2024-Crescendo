package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterSpeeds;

public class ShooterIdleSpeedCommand extends Command{

    private final ShooterSubsystem shooter;

    public ShooterIdleSpeedCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setSpeed(ShooterSpeeds.AMP);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setSpeed(ShooterSpeeds.OFF);
    }
}
