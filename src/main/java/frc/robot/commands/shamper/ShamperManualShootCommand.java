package frc.robot.commands.shamper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.subsystems.ShamperSubsystem.ShamperSpeed;

public class ShamperManualShootCommand extends Command{
    private final ShamperSubsystem shamper;
    private final ShamperSpeed goalSpeed;
    private final ShamperSpeed endSpeed;

    public ShamperManualShootCommand(ShamperSubsystem shamper, ShamperSpeed goalSpeed, ShamperSpeed endSpeed) {
        this.shamper = shamper;
        this.goalSpeed = goalSpeed;
        this.endSpeed = endSpeed;

        addRequirements(shamper);
    }

    public ShamperManualShootCommand(ShamperSubsystem shamper, ShamperSpeed goalSpeed) {
        this.shamper = shamper;
        this.goalSpeed = goalSpeed;
        this.endSpeed = ShamperSpeed.OFF;

        addRequirements(shamper);
    }

    @Override
    public void initialize() {
        shamper.setShootSpeed(goalSpeed);
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean interrupted) {
        shamper.stopShooter();
    }
}
