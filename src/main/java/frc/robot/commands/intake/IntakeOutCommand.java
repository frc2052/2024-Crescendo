package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeOutCommand extends Command {
    private final IntakeSubsystem intake;

    public IntakeOutCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.intakeOut();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}