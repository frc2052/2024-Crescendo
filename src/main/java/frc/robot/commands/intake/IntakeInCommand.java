package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeInCommand extends Command {
    private final IntakeSubsystem intake;

    public IntakeInCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.intakeIn();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}