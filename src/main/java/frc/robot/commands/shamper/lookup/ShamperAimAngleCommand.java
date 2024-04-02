package frc.robot.commands.shamper.lookup;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.util.AimingCalculator;
import frc.robot.util.calculator.ShootAngleConfig;
import frc.robot.util.calculator.ShootingAngleCalculator;

public class ShamperAimAngleCommand extends Command{
    private final ShamperSubsystem shamper;

    public ShamperAimAngleCommand(ShamperSubsystem shamper) {
        this.shamper = shamper;

        addRequirements(shamper);
    }

    @Override
    public void initialize() {
        RobotState.getInstance().updateIsVerticalAiming(true);
    }

    @Override
    public void execute() {
        ShootAngleConfig config = ShootingAngleCalculator.getInstance().getShooterConfig(AimingCalculator.calculateDistanceToSpeaker(RobotState.getInstance().getRobotPose()));
        shamper.setAngle(config.getAngleDegrees());
    }

    @Override
    public void end(boolean interrupted) {
        RobotState.getInstance().updateIsVerticalAiming(false);
    }
}
