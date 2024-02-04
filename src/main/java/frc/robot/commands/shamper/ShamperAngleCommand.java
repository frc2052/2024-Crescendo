package frc.robot.commands.shamper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShamperSubsystem;

public class ShamperAngleCommand extends Command {

    private final ShamperSubsystem shamper;
    private final double angle;

    public ShamperAngleCommand(ShamperSubsystem shamper, double angle) {
        this.shamper = shamper;
        this.angle = angle;
        addRequirements(shamper);
    }

    @Override
    public void initialize() {
        shamper.setAngle(angle);
    }
}
