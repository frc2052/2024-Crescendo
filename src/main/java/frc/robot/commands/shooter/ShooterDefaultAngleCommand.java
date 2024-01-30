package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterSpeeds;

public class ShooterDefaultAngleCommand extends Command{

    private final ShooterSubsystem shooter;

    public ShooterDefaultAngleCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setShooterRotationAngle(Constants.VerticalShooter.SHOOTER_DEFAULT_ANGLE);
    }
}
