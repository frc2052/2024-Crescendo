package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import frc.robot.Constants.Intake;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ForwardPixySubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.RobotStateEstimator;
import frc.robot.util.io.pixy.Pixy2CCC.Block;

public class AutoCenterLineNotePickupCommand extends DriveCommand {
    private final ForwardPixySubsystem pixy;

    private final PIDController yController;
    private final PIDController rotationController;
    private final Supplier<Rotation2d> goalRotation;

    private final double goalMeters;
    private final double backwardsSpeed;
    private final double sidewaysSpeed;
    private double startTime;
    
    private boolean isStage1;
    private boolean isStage2;
    private boolean isStage3;
    private boolean isStage4;

    private Pose2d startPose;

    private double centerLineXLocation = 16.459;

    public AutoCenterLineNotePickupCommand(
        double goalMeters,
        double backwardsSpeed,
        double sidewaysSpeed,
        Supplier<Rotation2d> goalRotation,
        DrivetrainSubsystem drivetrain,
        ForwardPixySubsystem pixy,
        IntakeSubsystem intake
    ) {
        super(() -> 0, () -> 0, () -> 0, () -> false, drivetrain);

        this.pixy = pixy;

        yController = new PIDController(.02, 0, 0);
        yController.setTolerance(30);
        yController.setSetpoint(-Constants.Intake.FRONT_PIXY_MOUNT_OFFSET_PIXELS);

        rotationController = new PIDController(2.5, 0, 0.1);
        rotationController.enableContinuousInput(0, 360);
        rotationController.setTolerance(2);
        
        this.goalMeters = goalMeters;
        this.backwardsSpeed = backwardsSpeed;
        this.sidewaysSpeed = sidewaysSpeed;
        this.goalRotation = goalRotation;

        addRequirements(pixy, drivetrain);
    }

    // STAGE 1: first attempt at pickup
    public boolean getStage1(){
        return isStage1;
    }

    public void setStage1(boolean isStage1){
        isStage1 = this.isStage1;
    }

    // STAGE 2: rotating to align onto center line
    public boolean getStage2(){
        return isStage2;
    }

    public void setStage2(boolean isStage2){
        isStage2 = this.isStage2;
    }

    // STAGE 3: moving down the center line
    public boolean getStage3(){
        return isStage3;
    }

    public void setStage3(boolean isStage3){
        isStage3 = this.isStage3;
    }

    // STAGE 4: moved past mid field, stop driving down center line (?)
    public boolean getStage4(){
        return isStage4;
    }

    public void setStage4(boolean isStage4){
        this.isStage4 = isStage4;
    }

    @Override
    public void initialize() {
        isStage1 = true;
        isStage2 = false;
        isStage3 = false;
        isStage4 = false;
        startPose = RobotState.getInstance().getRobotPose();
        startTime = Timer.getFPGATimestamp();
        rotationController.setSetpoint(goalRotation.get().getDegrees());
    };

    @Override
    protected double getY() {
        if (getStage1() || getStage3()){
        // if stages 1 or 3, move horizontally to align with note
            Block myFavoriteNote = pixy.findCentermostBlock();
            if(RobotState.getInstance().getNoteHeldDetected()) {
                return 0;
            } else if (myFavoriteNote == null) {
                return 0;
            } else {
                System.out.println("centermost block " + (myFavoriteNote.getX()));
                double yOffset = pixy.xOffsetFromCenter(myFavoriteNote);
                double ySpeed = -yController.calculate(yOffset) / 158;
                if(Math.abs(yOffset) < 25){
                    return 0;
                }
                if(Math.abs(ySpeed) > sidewaysSpeed){
                    ySpeed = Math.copySign(sidewaysSpeed, ySpeed);
                }
                return ySpeed;
            }
        // if stage 2 no horizontal movement so it can rotate       
        } else {
            return 0; 
        }
    }

    @Override
    protected double getX() {
        if(getStage1()){
        // during stage 1, if we have not yet reached the center line, drive backwards, then enter stage 2 (rotation)
            if (RobotState.getInstance().getRobotPose().getX() < centerLineXLocation){
                return backwardsSpeed;
            } else {
                setStage1(false);
                setStage2(true);
                setStage3(false);
                setStage4(false);
                return 0;
            }
        } else if (getStage2()){
            return 0;
        } else {
            // during stage 3, if robot sees note drive backwards until we intake that note
            Block myFavoriteNote = pixy.findCentermostBlock();
            if (myFavoriteNote != null) {
                return backwardsSpeed;
            } else {
                setStage1(false);
                setStage2(false);
                setStage3(false);
                setStage4(true);
                return 0;
            }
        }
    }

    @Override
    protected double getRotation(){
        if(getStage1()){
            return 0;
        } else if (getStage2()){
            double gyroDegrees = RobotState.getInstance().getRotation2d360().getDegrees();

            // Calculate PID value along with feedforward constant to assist with minor adjustments.
            double rotationValue = rotationController.calculate(gyroDegrees) / 360;
            if (!rotationController.atSetpoint()) {
                return rotationValue + Math.copySign(0.025, rotationValue);
            } else {
                setStage1(false);
                setStage2(false);
                setStage3(true);
                setStage4(false);
                return 0;
            }
        } else { 
            return 0;
        }
    }

    @Override
    protected boolean isFieldCentric(){
        return false;
    }

    @Override
    public boolean isFinished() {
        if (RobotState.getInstance().getNoteHeldDetected() || RobotState.getInstance().getNoteStagedDetected() || getStage4()){
            return true;
        } else {
            return false;
        }
    }
}

