package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    private final IntakeSubsystem intake;

    public IntakeCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.intake();
        //indexer.indexAll();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        //indexer.stop();
    }

    // @Override
    // public boolean isFinished() {
    //     return indexer.getNoteDetector();
    // }
}