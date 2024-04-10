package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState;
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
        RobotState.getInstance().updateIsIntaking(true);

    }

    @Override
    public void execute() {
        intake.intake();
        if(indexer.getNoteHeld() && !RobotState.getInstance().getNoteDetectorOverride()){
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
        RobotState.getInstance().updateIsIntaking(false);
    }

    @Override
    public boolean isFinished() {
        if(!RobotState.getInstance().getNoteDetectorOverride()){
            return indexer.getNoteStaged();
        } else {
            return false;
        }
    }
}