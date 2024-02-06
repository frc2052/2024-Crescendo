package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeInCommand extends Command {
    private final IntakeSubsystem intake;

    public IntakeInCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.intake(Constants.Intake.INTAKE_IN_SPEED_PCT, -Constants.Intake.INTAKE_IN_SPEED_PCT);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}