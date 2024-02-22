package frc.robot.commands.shamper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.subsystems.ShamperSubsystem.ShamperSpeed;
import frc.robot.util.calculator.ShootingAngleCalculator;

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

        //shamper.setAngle(ShootingAngleCalculator.getInstance().getShooterConfig(RobotState.getInstance().getRobotPose()).getAngleDegrees());
    }

    @Override
    public void end(boolean interrupted) {
        shamper.setShootSpeed(endSpeed);
        shamper.setAngle(Constants.Shamper.Angle.DEFAULT);
    }
}
