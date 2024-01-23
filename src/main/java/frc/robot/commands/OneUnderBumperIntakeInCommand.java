package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.OneUnderBumperIntakeSubsystem;

public class OneUnderBumperIntakeInCommand extends Command {
    private final OneUnderBumperIntakeSubsystem intake;

    public OneUnderBumperIntakeInCommand(OneUnderBumperIntakeSubsystem intake) {
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