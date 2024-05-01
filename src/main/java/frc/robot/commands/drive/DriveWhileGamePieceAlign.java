package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ForwardPixySubsystem;
import frc.robot.util.io.pixy.Pixy2CCC.Block;

public class DriveWhileGamePieceAlign extends DriveCommand {
    private final ForwardPixySubsystem pixy;

    private final PIDController rotationController;
    private DoubleSupplier rotationalSupplier;
    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;

    private Block myFavoriteNote;
    private boolean blind;

    private final double maxRotationalSpeed;
    private final double maxTranslationalSpeed;

    private Pose2d startPose;

    public DriveWhileGamePieceAlign(
        DoubleSupplier xSupplier, 
        DoubleSupplier ySupplier,  
        DoubleSupplier rotationalSupplier, 
        double maxRotationalSpeed,
        double maxTranslationalSpeed,
        DrivetrainSubsystem drivetrain,
        ForwardPixySubsystem pixy
    ) {
        super(()-> 0, () -> 0, () -> 0, () -> false, drivetrain);

        this.pixy = pixy;

        rotationController = new PIDController(0.5, 0, 0.005);
        rotationController.enableContinuousInput(-158, 158);
        rotationController.setTolerance(5);
        rotationController.setSetpoint(-Constants.Intake.FRONT_PIXY_MOUNT_OFFSET_PIXELS);

        this.rotationalSupplier = rotationalSupplier;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;

        this.maxRotationalSpeed = maxRotationalSpeed;
        this.maxTranslationalSpeed = maxTranslationalSpeed;

        myFavoriteNote = null;

        addRequirements(pixy, drivetrain);
    }

    @Override
    public void execute(){
        myFavoriteNote = pixy.findCentermostBlock();
        if(myFavoriteNote == null) {
            blind = true;
        } else { 
            blind = false;
        }
        super.execute();
    }

    @Override
    protected double getRotation() {
        if(blind || RobotState.getInstance().getNoteHeldDetected()) {
            System.out.println("no note :(");
            return -rotationalSupplier.getAsDouble();
        } else {
            double xOffset = pixy.xOffsetFromCenter(myFavoriteNote);
            double rotationSpeed = rotationController.calculate(xOffset) / 316;

            if (Math.abs(rotationSpeed) > maxRotationalSpeed){
                rotationSpeed = Math.copySign(maxRotationalSpeed, rotationSpeed);
            }

            return rotationSpeed;
        }
    }

    @Override
    protected double getX(){
        double yOffset = pixy.yOffsetFromTop(myFavoriteNote);

        if(blind || RobotState.getInstance().getNoteHeldDetected()) {
            return -xSupplier.getAsDouble();
        } else {
            double pctOff = Math.abs(yOffset) / 158 * 5;

            if (pctOff > 1) {
                pctOff = 1;
            }

            return -maxTranslationalSpeed * pctOff;
        }
    }

    @Override
    protected double getY(){
        double xOffset = pixy.xOffsetFromCenter(myFavoriteNote);

        if(blind || RobotState.getInstance().getNoteHeldDetected()) {
            return -ySupplier.getAsDouble();
        } else {
            double pctOff = xOffset / 158 * 5;

            if(pctOff > 1) {
                pctOff = 1;
            }

            return maxTranslationalSpeed * pctOff;
        }
    }

    @Override
    protected boolean isFieldCentric(){
        if(!blind){
            return false;
        } else {   
            return true;
        }
    }
}
