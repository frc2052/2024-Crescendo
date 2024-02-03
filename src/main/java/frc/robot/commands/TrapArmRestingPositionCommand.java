package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TrapArmSubsystem;
import frc.robot.subsystems.TrapArmSubsystem.TrapArmPositions;

public class TrapArmRestingPositionCommand extends Command {
    private static TrapArmSubsystem trapArm;

    public TrapArmRestingPositionCommand(TrapArmSubsystem trapArm) {
        this.trapArm = trapArm;
        addRequirements(trapArm);
    }

    @Override
    public void initialize() {
        trapArm.setPosition(TrapArmPositions.RESTING);
    }

    @Override
    public void end(boolean interrupted) {
        trapArm.setPosition(trapArm.getPosition());
    }
}
