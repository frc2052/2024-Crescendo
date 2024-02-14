package frc.robot.commands.shamper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.subsystems.ShamperSubsystem.ShamperSpeed;

public class ShamperShootCommand extends Command{
    private final ShamperSubsystem shamper;
    private final ShamperSpeed goalSpeed;
    private final ShamperSpeed endSpeed;
    private final IndexerSubsystem indexer;

    public ShamperShootCommand(ShamperSubsystem shamper, IndexerSubsystem indexer, ShamperSpeed goalSpeed, ShamperSpeed endSpeed) {
        this.shamper = shamper;
        this.goalSpeed = goalSpeed;
        this.endSpeed = endSpeed;
        this.indexer = indexer;

        addRequirements(shamper, indexer);
    }

    public ShamperShootCommand(ShamperSubsystem shamper, IndexerSubsystem indexer, ShamperSpeed goalSpeed) {
        this.shamper = shamper;
        this.goalSpeed = goalSpeed;
        this.endSpeed = goalSpeed;
        this.indexer = indexer;

        addRequirements(shamper, indexer);
    }

    @Override
    public void initialize() {
        indexer.index();
        shamper.setSpeed(goalSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stop();
        shamper.setSpeed(endSpeed);
    }
}
