package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotState;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ForwardPixySubsystem;
import frc.robot.util.io.pixy.Pixy2CCC.Block;

public class DriveWhileGamePieceAlign extends DriveCommand {
    private final ForwardPixySubsystem pixy;

    private final PIDController rotationController;

    private final double maxRotationalSpeed;

    private Pose2d startPose;

    public DriveWhileGamePieceAlign(
        DoubleSupplier xSupplier, 
        DoubleSupplier ySupplier, 
        double maxRotationalSpeed,
        DrivetrainSubsystem drivetrain,
        ForwardPixySubsystem pixy
    ) {
        super(xSupplier, ySupplier, () -> 0, () -> false, drivetrain);

        this.pixy = pixy;

        rotationController = new PIDController(1, 0, 0.025);
        rotationController.enableContinuousInput(-158, 158);
        // rotationController.setTolerance(3);
        rotationController.setSetpoint(-Constants.Intake.FRONT_PIXY_MOUNT_OFFSET_PIXELS);

        this.maxRotationalSpeed = maxRotationalSpeed;

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
            double rotationSpeed = rotationController.calculate(yOffset) / 316;
            // if(rotationController.atSetpoint()) {
            //     rotationSpeed = 0;
            // }

            if(Math.abs(rotationSpeed) > maxRotationalSpeed){
                rotationSpeed = Math.copySign(maxRotationalSpeed, rotationSpeed);
            }

            return rotationSpeed;
        }
    }

    @Override
    protected boolean isFieldCentric(){
        return true;
    }
}
