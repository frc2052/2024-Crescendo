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
import frc.robot.util.io.pixy.Pixy2CCC.Block;

public class GamePieceAlignmentCommand extends DriveCommand {
    private final ForwardPixySubsystem pixy;

    private final PIDController yController;

    private final double goalMeters;
    private final double backwardsSpeed;
    private final double sidewaysSpeed;

    private Pose2d startPose;

    public GamePieceAlignmentCommand(
        double goalMeters,
        double backwardsSpeed,
        double sidewaysSpeed,
        DrivetrainSubsystem drivetrain,
        ForwardPixySubsystem pixy
    ) {
        super(() -> 0, () -> 0, () -> 0, () -> false, drivetrain);

        this.pixy = pixy;

        yController = new PIDController(2.5, 0, 0.15);
        yController.enableContinuousInput(-158, 158);
        yController.setTolerance(10);
        yController.setSetpoint(-Constants.Intake.FRONT_PIXY_MOUNT_OFFSET_PIXELS);

        this.goalMeters = goalMeters;
        this.backwardsSpeed = backwardsSpeed;
        this.sidewaysSpeed = sidewaysSpeed;

        addRequirements(pixy, drivetrain);
    }

    @Override
    public void initialize() {
        startPose = RobotState.getInstance().getRobotPose();
    }

    @Override
    protected double getRotation() {
        Block myFavoriteNote = pixy.findCentermostBlock();
        if(myFavoriteNote == null || RobotState.getInstance().getNoteHeldDetected()) {
            System.out.println("no note :(");
            return 0;
        } else {
            double yOffset = pixy.xOffsetFromCenter(myFavoriteNote);
            double ySpeed = yController.calculate(yOffset) / 316;
            if(yController.atSetpoint()) {
                ySpeed = 0;
            }

            if(Math.abs(ySpeed) > sidewaysSpeed){
                ySpeed = Math.copySign(sidewaysSpeed, ySpeed);
            }

            return ySpeed;
        }
    }

    @Override
    protected double getX() {
        //System.out.println("ALIGNING X: " + drivetrain.getPosition().getX());
        return backwardsSpeed;
    }

    @Override
    protected boolean isFieldCentric(){
        return false;
    }

    @Override
    public boolean isFinished() {
        if (RobotState.getInstance().getRobotPose().getTranslation().getDistance(startPose.getTranslation()) > goalMeters){
            return true;
        } else if(RobotState.getInstance().getNoteStagedDetected()){
            return true;
        } else {
            return false;
        }
    }
}
