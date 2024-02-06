package frc.robot.commands.shamper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.subsystems.ShamperSubsystem.ShamperSpeeds;

public class ShamperShootCommand extends Command{

    private final ShamperSubsystem shamper;
    private final ShamperSpeeds speeds;

    public ShamperShootCommand(ShamperSubsystem shamper, ShamperSpeeds speeds) {
        this.shamper = shamper;
        this.speeds = speeds;
        addRequirements(shamper);
    }

    @Override
    public void initialize() {
        shamper.setSpeed(speeds);
    }
}
