package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ForwardPixySubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class GamePieceAlignmentCommand extends DriveCommand {
    private final ForwardPixySubsystem pixy;
    private final IntakeSubsystem intake;

    private final PIDController xController;
    private final PIDController yController;

    private final DoubleSupplier goalXMeters;

    public GamePieceAlignmentCommand(
        DoubleSupplier goalXMeters,
        DrivetrainSubsystem drivetrain,
        ForwardPixySubsystem pixy,
        IntakeSubsystem intake
    ) {
        super(() -> 0, () -> 0, () -> 0, () -> false, drivetrain);

        this.pixy = pixy;
        this.intake = intake;

        xController = new PIDController(0.5, 0, 0);
        xController.setTolerance(0.1);

        yController = new PIDController(1, 0, 0);
        yController.setTolerance(5);
        yController.setSetpoint(-Constants.Intake.FRONT_PIXY_MOUNT_OFFSET_PIXELS);

        this.goalXMeters = goalXMeters;

        addRequirements(pixy, drivetrain);
    }

    @Override
    public void initialize() {
        xController.setSetpoint(goalXMeters.getAsDouble());
    }

    @Override
    protected double getY() {
        double yOffset = pixy.xOffsetFromCenter(pixy.findCentermostBlock());

        return yController.calculate(yOffset) / 158;
    }

    // @Override
    // protected double getX() {
    //     //System.out.println("ALIGNING X: " + drivetrain.getPosition().getX());
    //     if (goalXMeters.getAsDouble() != 0) {
    //         return xController.calculate(drivetrain.getPosition().getX());
    //     } else {
    //         return 0;
    //     }
    // }

    @Override
    public boolean isFinished() {
        if (goalXMeters.getAsDouble() == 0) {
            return false;
        }
        return  xController.atSetpoint(); // || intake.isCurrentLimiting();
    }
}
