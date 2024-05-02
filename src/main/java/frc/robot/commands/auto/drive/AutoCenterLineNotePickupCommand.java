package frc.robot.commands.auto.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ForwardPixySubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.io.pixy.Pixy2CCC.Block;

public class AutoCenterLineNotePickupCommand extends AutoDriveWhileGamePieceAlign {

    private final PIDController yController;
    private final PIDController rotationController;
    private final Supplier<Rotation2d> goalRotation;

    // private final double goalMeters;
    // private final double backwardsSpeed;
    // private final double sidewaysSpeed;
    private double startTime;
    
    private boolean isStage1;
    private boolean isStage2;
    private boolean isStage3;
    private boolean isStage4;

    private Pose2d startPose;

    private double centerLineXLocation = 16.459;

    public int stage = 1;

    public AutoCenterLineNotePickupCommand (
        double maxRotationalSpeed,
        double maxTranslationalSpeed,
        Supplier<Rotation2d> goalRotation,
        DrivetrainSubsystem drivetrain,
        ForwardPixySubsystem pixy,
        IntakeSubsystem intake
    ) {
        super(maxRotationalSpeed, maxTranslationalSpeed, 0, drivetrain, pixy);

        yController = new PIDController(.02, 0, 0);
        yController.setTolerance(30);
        yController.setSetpoint(-Constants.Intake.FRONT_PIXY_MOUNT_OFFSET_PIXELS);

        rotationController = new PIDController(2.5, 0, 0.1);
        rotationController.enableContinuousInput(0, 360);
        rotationController.setTolerance(2);
        
        // this.goalMeters = goalMeters;
        // this.backwardsSpeed = backwardsSpeed;
        // this.sidewaysSpeed = sidewaysSpeed;
        this.goalRotation = goalRotation;

        // addRequirements(pixy, drivetrain);
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
        stage = 1;
        System.out.println("stage 1");
        startPose = RobotState.getInstance().getRobotPose();
        startTime = Timer.getFPGATimestamp();
        rotationController.setSetpoint(goalRotation.get().getDegrees());
    };

    @Override
    public void execute(){
        System.out.println("stage = " + stage);
        super.execute();
    }

    @Override
    protected double getY() {
        if (stage == 1 || stage == 3){
        // if stages 1 or 3, move horizontally to align with note
            return super.getY();
            // Block myFavoriteNote = pixy.findCentermostBlock();
            // if(RobotState.getInstance().getNoteHeldDetected()) {
            //     return 0;
            // } else if (myFavoriteNote == null) {
            //     return 0;
            // } else {
            //     System.out.println("centermost block " + (myFavoriteNote.getX()));
            //     double yOffset = pixy.xOffsetFromCenter(myFavoriteNote);
            //     double ySpeed = -yController.calculate(yOffset) / 158;
            //     if(Math.abs(yOffset) < 25){
            //         return 0;
            //     }
            //     if(Math.abs(ySpeed) > sidewaysSpeed){
            //         ySpeed = Math.copySign(sidewaysSpeed, ySpeed);
            //     }
            //     return ySpeed;
            // }
        // if stage 2 no horizontal movement so it can rotate       
        } else {
            return 0; 
        }
    }

    @Override
    protected double getX() {
        if(stage == 1){
            System.out.println(RobotState.getInstance().getRobotPose().getX());
        // during stage 1, if we have not yet reached the center line, drive backwards, then enter stage 2 (rotation)
            if (RobotState.getInstance().getRobotPose().getX() < centerLineXLocation){
                //return backwardsSpeed;
                return super.getX();
            } else {
                stage = 2;
                return 0;
            }
        } else if (stage == 2){
            return 0;
        } else {
            // during stage 3, if robot sees note drive backwards until we intake that note
            // Block myFavoriteNote = pixy.findCentermostBlock();
            if (stage == 3) {
                // return backwardsSpeed;
                return super.getX();
            } else {
                stage = 4;
                return 0;
            }
        }
    }

    @Override
    protected double getRotation(){
        if(getStage1()){
            // return 0;
            return super.getRotation();
        } else if (getStage2()){
            double gyroDegrees = RobotState.getInstance().getRotation2d360().getDegrees();

            // Calculate PID value along with feedforward constant to assist with minor adjustments.
            double rotationValue = rotationController.calculate(gyroDegrees) / 360;
            if (!rotationController.atSetpoint()) {
                return rotationValue + Math.copySign(0.025, rotationValue);
            } else {
                stage = 2;
                return 0;
            }
        } else { 
            // return 0;
            return super.getRotation();
        }
    }

    @Override
    protected boolean isFieldCentric(){
        return false;
    }

    @Override
    public boolean isFinished() {
        if (RobotState.getInstance().getNoteHeldDetected() || RobotState.getInstance().getNoteStagedDetected() || stage == 4){
            return true;
        } else {
            return false;
        }
    }
}

