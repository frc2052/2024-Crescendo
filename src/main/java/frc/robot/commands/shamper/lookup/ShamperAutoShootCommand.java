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

public class ShamperAutoShootCommand extends Command{
    private final ShamperSubsystem shamper;
    private final IndexerSubsystem indexer;

    public ShamperAutoShootCommand(ShamperSubsystem shamper, IndexerSubsystem indexer) {
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
        Logger.recordOutput("shoot speed calculated", config.getShooterSpeedVelocityRPS());
        Logger.recordOutput("angle calculated", config.getAngleDegrees());
        shamper.setShootSpeed(config.getShooterSpeedVelocityRPS(), config.getShooterSpeedVelocityRPS());
        if(shamper.shooterAtSpeed(config.getShooterSpeedVelocityRPS(), config.getShooterSpeedVelocityRPS()) && shamper.isAtGoalAngle()) {
            indexer.indexAll();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shamper.windDownShooter();
        indexer.stop();
    }
}