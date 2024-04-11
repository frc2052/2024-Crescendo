package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.team2052.lib.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.AimingCalculator;
import frc.robot.util.io.Dashboard;
import frc.robot.util.states.DrivetrainState;

public class RobotState {
    private static RobotState INSTANCE;

    private Pose2d initialPose;
    private Pose2d robotPose;
    private Pose3d aprilTagVisionPose3d;
    private List<PhotonTrackedTarget> activeTargets;
    private boolean visionEnabled;
    private double detectionTime;
    private Rotation2d navxRotation;
    private SwerveModulePosition[] swerveModulePositions;
    private ChassisSpeeds chassisSpeeds;
    private boolean noteHeld;
    private boolean noteStaged;
    private boolean musicEnabled;
    private boolean isClimbing;
    private boolean isLobbing;
    private boolean shamperAtGoalAngle;
    private boolean noteDetectorOverride;

    private boolean isShooting;
    private boolean atGoalRotation;
    private boolean isVerticalAiming;
    private boolean isHorizontalAiming;
    private boolean isIntaking;
    private boolean isAmpIdle;

    public static RobotState getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new RobotState();
        }

        return INSTANCE;
    }

    private RobotState() {
        initialPose = new Pose2d();
        robotPose = new Pose2d();
        detectionTime = 0.0;
        aprilTagVisionPose3d = new Pose3d();
        activeTargets = new ArrayList<PhotonTrackedTarget>();
        visionEnabled = false;
        navxRotation = new Rotation2d(0);
        chassisSpeeds = new ChassisSpeeds();
        noteHeld = false;
        noteStaged = false;
        noteDetectorOverride = false;
        isLobbing = false;

        isShooting = false;
        atGoalRotation = false;
        isVerticalAiming = false;
        isHorizontalAiming = false;
        isIntaking = false;
        isAmpIdle = false;
    }  

    public boolean hasValidSwerveState() {
        return swerveModulePositions != null;
    }

    public void addDrivetrainState(DrivetrainState drivetrainState) {
        this.chassisSpeeds = drivetrainState.getChassisSpeeds();
        this.swerveModulePositions = drivetrainState.getModulePositions();
        this.navxRotation = drivetrainState.getRotation2d();
    }

    public Rotation2d getGyroRotation() {
        return navxRotation;
    }

    /**
     * Adds an AprilTag vision update
     */ 
    public void addAprilTagVisionUpdate(EstimatedRobotPose aprilTagVisionPose) {
        if(!(aprilTagVisionPose.estimatedPose.getTranslation() == new Translation3d())){
            
            // check if any of the targets have a ambiguity greater than 0.2 to filter out tags that could potentially throw us across the field
            for (PhotonTrackedTarget target : aprilTagVisionPose.targetsUsed) {
                if(target.getPoseAmbiguity() > 0.25 && target.getPoseAmbiguity() > 0){
                    this.aprilTagVisionPose3d = null;
                    this.detectionTime = 0;
                    return;
                }
            }
            this.aprilTagVisionPose3d = aprilTagVisionPose.estimatedPose;
            this.detectionTime = aprilTagVisionPose.timestampSeconds;
            this.activeTargets = aprilTagVisionPose.targetsUsed;
        } else {
            System.out.println("EMPTY VISION POSE");
            this.aprilTagVisionPose3d = null;
            this.detectionTime = 0;
        }
    }

    public List<PhotonTrackedTarget> getActiveTargets() {
        return activeTargets;
    }

    public void updateVisionEnabled(boolean visionEnabled) {
        this.visionEnabled = visionEnabled;
        Dashboard.getInstance().putData("vision enabled", visionEnabled);
    }

    public boolean getVisionEnabled() {
        return visionEnabled;
    }

    public void updateRobotPose(Pose2d robotPose) {
        this.robotPose = robotPose;
    }

    public void updateNoteDetectorOverride(boolean override) {
        this.noteDetectorOverride = override;
    }

    public boolean getNoteDetectorOverride(){
        return noteDetectorOverride;
    }
    public void updateNoteHeld(boolean noteHeld) {
        this.noteHeld = noteHeld;
    }

    public void updateNoteStaged(boolean noteStaged) {
        this.noteStaged = noteStaged;
    }

    public void updateIsClimbing(boolean isClimbing) {
        this.isClimbing = isClimbing;
    }
    
    public boolean getIsClimbing(){
        return isClimbing;
    }

    public void setIsLobbing(boolean isLobbing) {
        this.isLobbing = isLobbing;
    }

    public boolean getIsLobbing() {
        return isLobbing;
    }

    public void updateShamperAtGoalAngle(boolean shamperAtGoalAngle) {
        this.shamperAtGoalAngle = shamperAtGoalAngle;
    }

    public boolean getIsShamperAtGoalAngle() {
        return shamperAtGoalAngle;
    }

     public void updateShooting(boolean isShooting){
        this.isShooting = isShooting;
    }

    public boolean getShooting(){
        return isShooting;
    }

    public void updateAmpIdle(boolean isAmpIdle) {
        this.isAmpIdle = isAmpIdle;
    }

    public boolean getAmpIdle(){
        return isAmpIdle;
    }

     public void updateRotationOnTarget(boolean isOnTarget){
        this.atGoalRotation = isOnTarget;
    }

    public boolean getIsRotationOnTarget(){
        return atGoalRotation;
    }

    public void updateIsVerticalAiming(boolean isVerticalAiming){
        this.isVerticalAiming = isVerticalAiming;
    }

   public boolean getIsVerticalAiming(){
        return isVerticalAiming;
   }

   public void updateIsHorizontalAiming(boolean isHorizontalAiming){
        this.isHorizontalAiming = isHorizontalAiming;
   }

   public boolean getIsHorizontalAiming(){
        return isHorizontalAiming;
   }

   public void updateIsIntaking(boolean isIntaking){
        this.isIntaking = isIntaking;
   }

   public boolean getIsIntaking(){
        return isIntaking;
   }



    /**
     * Reset the RobotState's Initial Pose2d and set the NavX Offset. 
     * NavX offset is set when the robot has an inital rotation not facing where you want 0 (forwards) to be.
     */
    public void resetInitialPose(Pose2d initialStartingPose) {
        //navxOffset = new Rotation2d();
        //autoOffset = -initialStartingPose.getRotation().getRadians() + Math.PI;
        //System.out.println("auto offset of :" + Units.radiansToDegrees(autoOffset));
        initialPose = initialStartingPose;
    }

    /**
     * Returns the latest AprilTag vision detection robot translation in Translation2d
     * 
     * @return Translation2d
     */
    public Optional<Pose3d> getVisionPose3d() {
        Optional<Pose3d> visionPose = Optional.empty();

        // if the vision pose doesn't have it's pose at the origin and not null, then it's good
        if(!(aprilTagVisionPose3d == null)){
            if (!(aprilTagVisionPose3d.getTranslation() == new Translation3d())){
                visionPose = Optional.of(aprilTagVisionPose3d);
            }
        }

        return visionPose;
    }

    /**
     * Returns the latest AprilTag Vision detection time. This is when (on the raspberry pi) the Translation3d was last updated.
     * 
     * @return double
     */
    public double getVisionDetectionTime() {
        if(detectionTime == 0.0){
            return Timer.getFPGATimestamp();
        } else {
            return detectionTime;
        }
    }

    /**
     * Returns the Rotation2d of the robot, accounting for an offset that was set when intialized. 
     * 
     * @return Rotation2d
     */
    public Rotation2d getRotation2d180() {
        double rotationDegrees = MathUtil.inputModulus(getRotation2d().getDegrees(), -180, 180);

        return Rotation2d.fromDegrees(rotationDegrees);
    }

    public Rotation2d getRotation2d360() {
        double rotationDegrees = MathUtil.inputModulus(getRotation2d().getDegrees(), 0, 360);
        return Rotation2d.fromDegrees(rotationDegrees);
    }

    public Rotation2d getRotation2d() {
        return robotPose.getRotation();
    }

    /**
     * Returns the SwerveModulePositions of the drivetrain swerve modules. 
     * 
     * @return SwerveModulePosition[]
     */
    public SwerveModulePosition[] getModulePositions() {
        return swerveModulePositions;
    }
    
    /**
     * Returns the ChassisSpeeds of the chassis based off kinematics. This is robot relative
     * 
     * @return ChassisSpeeds
     */
    public ChassisSpeeds getChassisSpeeds() {
        return chassisSpeeds;
    }

    /**
     * Returns the Pose2d of the robot that was given by the estimator. 
     * 
     * @return Pose2d
     */
    public Pose2d getRobotPose() {
        return robotPose;
    }

    public Pose2d getRobotPoseAuto(){
        // return new Pose2d(robotPose.getTranslation(), getRotation2d180());
        return robotPose;
    }

    public boolean getNoteStagedDetected() {
        return noteStaged;
    }

    public boolean getNoteHeldDetected() {
        return noteHeld;
    }

    /**
     * Returns the initial Pose2d of the robot since last reset.
     * 
     * @return Pose2d
     */
    public Pose2d getInitialPose() {
        return initialPose;
    }

    public boolean getMusicEnableStatus() {
        return musicEnabled;
    }    
    
    public void setMusicEnableStatus(boolean isEnabled) {
        musicEnabled = isEnabled;
    }

    public boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return (alliance.get() == DriverStation.Alliance.Red) ? true : false;
        } else {
            return false;
        }
    }

    public Translation2d getSpeakerLocation() {
        if(isRedAlliance()) {
            return Constants.FieldAndRobot.RED_SPEAKER_LOCATION;
        } else {
            return Constants.FieldAndRobot.BLUE_SPEAKER_LOCATION;
        }
    }

    public Translation2d getSpeakerAimLocation() {
        if(isRedAlliance()) {
            return Constants.FieldAndRobot.RED_SPEAKER_AIM_LOCATION;
        } else {
            return Constants.FieldAndRobot.BLUE_SPEAKER_AIM_LOCATION;
        }
    }

    public double distanceToSpeaker() {
        return AimingCalculator.calculateDistanceToSpeaker(robotPose);
    }

    /**
     * Returns true if the robot is disabled.
     *
     * @return True if the robot is disabled.
     */
    public static boolean isDisabled() {
        return DriverStation.isDisabled();
    }

    /**
     * Returns true if the robot is enabled.
     *
     * @return True if the robot is enabled.
     */
    public static boolean isEnabled() {
        return DriverStation.isEnabled();
    }

    /**
     * Returns true if the robot is E-stopped.
     *
     * @return True if the robot is E-stopped.
     */
    public static boolean isEStopped() {
        return DriverStation.isEStopped();
    }

    /**
     * Returns true if the robot is in teleop mode.
     *
     * @return True if the robot is in teleop mode.
     */
    public static boolean isTeleop() {
        return DriverStation.isTeleop();
    }

    /**
     * Returns true if the robot is in autonomous mode.
     *
     * @return True if the robot is in autonomous mode.
     */
    public static boolean isAutonomous() {
        return DriverStation.isAutonomous();
    }

    /**
     * Returns true if the robot is in test mode.
     *
     * @return True if the robot is in test mode.
     */
    public static boolean isTest() {
        return DriverStation.isTest();
    }



    public void output(){
        Logger.recordOutput("Vision Pose", aprilTagVisionPose3d);
        Logger.recordOutput("Robot Position X : ", (robotPose.getX()));
        Logger.recordOutput("Robot Position Y : ", (robotPose.getY()));
        Logger.recordOutput("ROBOT POSE2D", robotPose);
        Logger.recordOutput("Auto Pose", getRobotPoseAuto());
        Logger.recordOutput("distance calculated hypot", AimingCalculator.calculateAimPointSpeaker(robotPose));
        Logger.recordOutput("RAW GYRO", navxRotation.getDegrees());
        Logger.recordOutput("Robot Rotation", robotPose.getRotation().getDegrees());
        Logger.recordOutput("NOTE STAGED", noteStaged);
        Logger.recordOutput("NOTE HELD", noteHeld);
        Logger.recordOutput("auto gyro method angle", robotPose.getRotation().getDegrees());
        Dashboard.getInstance().putData("NOTE HELD", noteHeld);
        Dashboard.getInstance().updateIsClimbing(isClimbing);
    }   
}
