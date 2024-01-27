package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterSpeeds;

public class ShooterAmpSpeedCommand extends Command{

    private final ShooterSubsystem shooter;

    public ShooterAmpSpeedCommand(ShooterSubsystem shooter) {
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
