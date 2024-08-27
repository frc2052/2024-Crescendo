package frc.robot.commands.autonomous.drive;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.RobotState;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ForwardPixySubsystem;
import frc.robot.util.io.pixy.Pixy2CCC.Block;

public class AutoDriveWhileGamePieceAlign extends DriveCommand{
    private final ForwardPixySubsystem pixy;

    private final PIDController rotationController;

    private Block myFavoriteNote;
    private boolean blind;

    private final double maxRotationalSpeed;
    private final double maxTranslationalSpeed;
    private final double blindSpeed;

    public AutoDriveWhileGamePieceAlign(
        double maxRotationalSpeed,
        double maxTranslationalSpeed,
        double blindSpeed,
        DrivetrainSubsystem drivetrain,
        ForwardPixySubsystem pixy
    ) {
        super(() -> 0, () -> 0, () ->0, () -> false, drivetrain);

        this.pixy = pixy;

        rotationController = new PIDController(0.5, 0, 0.005);
        rotationController.enableContinuousInput(-158, 158);
        rotationController.setTolerance(5);
        rotationController.setSetpoint(-Constants.Intake.FRONT_PIXY_MOUNT_OFFSET_PIXELS);

        this.maxRotationalSpeed = maxRotationalSpeed;
        this.maxTranslationalSpeed = maxTranslationalSpeed;
        this.blindSpeed = blindSpeed;

        myFavoriteNote = null;

        addRequirements(pixy);
    }

    @Override
    public void execute() {
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
            return 0;
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

        if(blind) {
            return -blindSpeed;
        } else if (RobotState.getInstance().getNoteHeldDetected()){
            return 0;
        } else {
            double pctOff = Math.abs(yOffset) / 158 * 3;

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
            return 0;
        } else {
            double pctOff = xOffset / 158;

            if(pctOff > 1) {
                pctOff = 1;
            }

            return maxTranslationalSpeed * pctOff;
        }
    }

    @Override
    protected boolean isFieldCentric(){
        return false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
