package frc.robot.commands.autonomous.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeCommand extends Command {
    private final IntakeSubsystem intake;
    private final IndexerSubsystem indexer;
    // private final ShamperSubsystem shamper;

    public AutoIntakeCommand(IntakeSubsystem intake, IndexerSubsystem indexer) {
        this.intake = intake;
        this.indexer = indexer;
        // this.shamper = shamper;
        addRequirements(intake, indexer);
    }
    @Override
    public void initialize(){
        //shamper.windDownShooter();
        // shamper.setAngle(Constants.Shamper.Angle.AUTO_INTAKE);
    }

    @Override
    public void execute() {
        intake.intake();
        if(indexer.getNoteHeld()){
            indexer.loadSlow();
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