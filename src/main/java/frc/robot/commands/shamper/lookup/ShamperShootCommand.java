package frc.robot.commands.shamper.lookup;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.util.calculator.ShootAngleConfig;
import frc.robot.util.calculator.ShootingAngleCalculator;

public class ShamperShootCommand extends Command{
    private final ShamperSubsystem shamper;
    private final IndexerSubsystem indexer;

    public ShamperShootCommand(ShamperSubsystem shamper, IndexerSubsystem indexer) {
        this.shamper = shamper;
        this.indexer = indexer;

        addRequirements(shamper);
    }

    @Override
    public void initialize() {
        RobotState.getInstance().updateShooting(true);
    }

    @Override
    public void execute() {
        // ShootAngleConfig config = ShootingAngleCalculator.getInstance().getShooterConfig(AimingCalculator.calculateDistanceToAimPoint(RobotState.getInstance().getRobotPose()));
        ShootAngleConfig config = ShootingAngleCalculator.getInstance().getShooterConfig(RobotState.getInstance().distanceToSpeaker());

        shamper.setShootSpeed(config.getLowerShooterSpeedVelocityRPS(), config.getUpperShooterSpeedVelocityRPS());
        if(shamper.shooterAtSpeed(config.getLowerShooterSpeedVelocityRPS(), config.getUpperShooterSpeedVelocityRPS()) && shamper.isAtGoalAngle()) {
            indexer.indexAll();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shamper.windDownShooter();
        indexer.stop();
        RobotState.getInstance().updateShooting(false);
    }
}