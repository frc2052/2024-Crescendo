package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AimingCalculator;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterSpeeds;

public class ShooterSpeakerAngleMovingCommand extends Command{

    private final ShooterSubsystem shooter;

    public ShooterSpeakerAngleMovingCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        AimingCalculator.updateInformation();
        shooter.setShooterRotationAngle(AimingCalculator.getMovingTargetShooterAngle());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterRotationAngle(Constants.VerticalShooter.SHOOTER_DEFAULT_ANGLE);
    }
}