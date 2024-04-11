package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotState;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ForwardPixySubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.RobotStateEstimator;

public class GamePieceAlignmentCommand extends DriveCommand {
    private final ForwardPixySubsystem pixy;

    private final PIDController yController;

    private final double goalMeters;
    private final double xSpeed;
    private Pose2d startPose;

    public GamePieceAlignmentCommand(
        double goalMeters,
        double xSpeed,
        DrivetrainSubsystem drivetrain,
        ForwardPixySubsystem pixy
    ) {
        super(() -> 0, () -> 0, () -> 0, () -> false, drivetrain);

        this.pixy = pixy;

        yController = new PIDController(1, 0, 0);
        yController.setTolerance(5);
        yController.setSetpoint(-Constants.Intake.FRONT_PIXY_MOUNT_OFFSET_PIXELS);

        this.goalMeters = goalMeters;
        this.xSpeed = xSpeed;

        addRequirements(pixy, drivetrain);
    }

    @Override
    public void initialize() {
        startPose = RobotState.getInstance().getRobotPose();
    }

    @Override
    protected double getY() {
        double yOffset = pixy.xOffsetFromCenter(pixy.findCentermostBlock());

        return yController.calculate(yOffset) / 158;
    }

    @Override
    protected double getX() {
        //System.out.println("ALIGNING X: " + drivetrain.getPosition().getX());
        return xSpeed;
    }

    @Override
    public boolean isFinished() {
        if (RobotState.getInstance().getRobotPose().getTranslation().getDistance(startPose.getTranslation()) > goalMeters){
            return true;
        } else if(RobotState.getInstance().getNoteHeldDetected()){
            return true;
        } else {
            return false;
        }
    }
}
