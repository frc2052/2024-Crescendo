package frc.robot.commands.shamper;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.subsystems.ShamperSubsystem.ShamperSpeed;

public class ShamperShootCommand extends Command{
    private final ShamperSubsystem shamper;
    private final ShamperSpeed goalSpeed;
    private final ShamperSpeed endSpeed;
    private final IndexerSubsystem indexer;
    private final DoubleSupplier targetAngle;

    public ShamperShootCommand(ShamperSubsystem shamper, IndexerSubsystem indexer, ShamperSpeed goalSpeed, ShamperSpeed endSpeed, DoubleSupplier targetAngle) {
        this.shamper = shamper;
        this.goalSpeed = goalSpeed;
        this.endSpeed = endSpeed;
        this.indexer = indexer;
        this.targetAngle = targetAngle;

        addRequirements(shamper);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shamper.setShootSpeed(goalSpeed);
        shamper.setAngle(targetAngle.getAsDouble());
        if(shamper.shooterAtSpeed(goalSpeed.getLowerRPS(), goalSpeed.getUpperRPS()) && 
        (shamper.getShamperAngle() == targetAngle.getAsDouble())) {
            indexer.indexAll();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shamper.setShootSpeed(endSpeed);
        shamper.setAngle(Constants.Shamper.Angle.DEFAULT);
    }
}
