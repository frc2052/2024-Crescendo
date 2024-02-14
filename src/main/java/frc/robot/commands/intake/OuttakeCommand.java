package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class OuttakeCommand extends Command {
    private final IntakeSubsystem intake;

    public OuttakeCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.intake();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}