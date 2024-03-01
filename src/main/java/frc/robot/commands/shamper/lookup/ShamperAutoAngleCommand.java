package frc.robot.commands.shamper.lookup;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.subsystems.ShamperSubsystem.ShamperSpeed;
import frc.robot.util.AimingCalculator;
import frc.robot.util.calculator.ShootAngleConfig;
import frc.robot.util.calculator.ShootingAngleCalculator;

public class ShamperAutoAngleCommand extends Command{
    private final ShamperSubsystem shamper;
    private final IndexerSubsystem indexer;

    public ShamperAutoAngleCommand(ShamperSubsystem shamper, IndexerSubsystem indexer) {
        this.shamper = shamper;
        this.indexer = indexer;

        addRequirements(shamper);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        ShootAngleConfig config = ShootingAngleCalculator.getInstance().getShooterConfig(AimingCalculator.calculateDistanceToSpeaker(RobotState.getInstance().getRobotPose()));
        shamper.setAngle(config.getAngleDegrees());
    }

    @Override
    public void end(boolean interrupted) {
    }
}
