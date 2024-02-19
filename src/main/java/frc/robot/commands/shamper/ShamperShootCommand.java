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
        this.endSpeed = ShamperSpeed.OFF;
        this.indexer = indexer;

        addRequirements(shamper, indexer);
    }

    @Override
    public void initialize() {
        shamper.setShootSpeed(goalSpeed);
    }

    @Override
    public void execute() {
        if(shamper.shooterAtSpeed(goalSpeed.getLowerPCT(), goalSpeed.getUpperPCT())) {
            indexer.indexAll();
        }
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stop();
        shamper.setShootSpeed(endSpeed);
    }
}
