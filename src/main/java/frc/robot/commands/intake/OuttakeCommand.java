package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShamperSubsystem;

public class OuttakeCommand extends Command {
    private final IntakeSubsystem intake;
    private final IndexerSubsystem indexer;
    private final ShamperSubsystem shamper;

    public OuttakeCommand(IntakeSubsystem intake, IndexerSubsystem indexer, ShamperSubsystem shamper) {
        this.intake = intake;
        this.indexer = indexer;
        this.shamper = shamper;
        addRequirements(intake, indexer, shamper);
    }
    @Override
    public void initialize(){
        shamper.setAngle(Constants.Shamper.Angle.DEFAULT);
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

    @Override
    public boolean isFinished() {
        return false;
    }
}