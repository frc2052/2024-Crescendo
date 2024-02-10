package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeInCommand extends Command {
    private final IntakeSubsystem intake;
    private final IndexerSubsystem indexer;
    private final DigitalInput digitalInput;

    public IntakeInCommand(IntakeSubsystem intake, IndexerSubsystem indexer) {
        this.intake = intake;
        this.indexer = indexer;
        digitalInput = new DigitalInput(Constants.Indexer.DIGITAL_INPUT_ID);
        addRequirements(indexer, intake);
    }

    @Override
    public void initialize() {
        intake.intake(Constants.Intake.INTAKE_IN_SPEED_PCT, -Constants.Intake.INTAKE_IN_SPEED_PCT);
        indexer.runMotors();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        indexer.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return digitalInput.get();
    }
}