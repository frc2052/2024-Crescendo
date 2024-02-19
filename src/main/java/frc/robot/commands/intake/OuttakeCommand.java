package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class OuttakeCommand extends Command {
    private final IntakeSubsystem intake;
    private final IndexerSubsystem indexer;

    public OuttakeCommand(IntakeSubsystem intake, IndexerSubsystem indexer) {
        this.intake = intake;
        this.indexer = indexer;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.outtake();
        indexer.reverse();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        indexer.stop();
    }
}