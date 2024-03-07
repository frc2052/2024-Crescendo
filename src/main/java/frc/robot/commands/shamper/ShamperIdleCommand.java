package frc.robot.commands.shamper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.subsystems.ShamperSubsystem.ShamperSpeed;

public class ShamperIdleCommand extends Command{
    private final ShamperSubsystem shamper;

    public ShamperIdleCommand(ShamperSubsystem shamper) {
        this.shamper = shamper;

        addRequirements(shamper);
    }

    @Override
    public void initialize() {
        shamper.setShootSpeed(ShamperSpeed.SPEAKER_IDLE);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        shamper.stopShooter();
    }
}
