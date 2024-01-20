package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.HorizontalShooterSubsystem;

public class HorizontalShooterCommand extends Command {
    private final HorizontalShooterSubsystem shooter;

    public HorizontalShooterCommand(HorizontalShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setLeftShooterSpeed(Constants.HorizontalShooter.SHOOTER_SHOOT_SPEED_TPS);
        shooter.setRightShooterSpeed(Constants.HorizontalShooter.SHOOTER_SHOOT_SPEED_TPS);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}
