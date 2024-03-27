package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShamperSubsystem;

public class IntakeCommand extends Command {
    private final IntakeSubsystem intake;
    private final IndexerSubsystem indexer;
    private final ShamperSubsystem shamper;

    public IntakeCommand(IntakeSubsystem intake, IndexerSubsystem indexer, ShamperSubsystem shamper) {
        this.intake = intake;
        this.indexer = indexer;
        this.shamper = shamper;
        addRequirements(intake, indexer, shamper);
    }
    @Override
    public void initialize(){
        //shamper.windDownShooter();
        shamper.setAngle(Constants.Shamper.Angle.INTAKE);
    }

    @Override
    public void execute() {
        intake.intake();
        if(indexer.getNoteHeld()){
            indexer.loadSlow();
            intake.outtake();
        } else {
            indexer.load();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        indexer.stop();
    }

    @Override
    public boolean isFinished() {
        return indexer.getNoteStaged();
    }
}