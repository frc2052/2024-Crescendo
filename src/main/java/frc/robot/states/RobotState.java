package frc.robot.states;

import com.team2052.lib.DrivetrainState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.io.Dashboard;
import frc.robot.Constants;
import frc.robot.states.Superstructure.SuperstructureState;

public class RobotState {
    private static RobotState INSTANCE;

    private Pose2d initialPose;
    private Pose2d robotPose;
    private Pose2d robotVisionPose2d;
    private double detectionTime;
    private Rotation2d navxOffset;
    private Rotation2d robotRotation2d;
    private SwerveModulePosition[] swerveModulePositions;
    private ChassisSpeeds chassisSpeeds;
    private boolean noteDetected;

    private SuperstructureState superstructureState;

    public static RobotState getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new RobotState();
        }

        return INSTANCE;
    }

    public boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return (alliance.get() == DriverStation.Alliance.Red) ? 
            true : false;
        } else {
            return false;
        }
    }

    private RobotState() {
        initialPose = new Pose2d();
        robotPose = new Pose2d();
        detectionTime = 0.0;
        robotVisionPose2d = new Pose2d();
        navxOffset = new Rotation2d(0);
        robotRotation2d = new Rotation2d(0);
        chassisSpeeds = new ChassisSpeeds();
        noteDetected = false;
    }  

    public boolean hasValidSwerveState() {
        return swerveModulePositions != null;
    }

    public void addDrivetrainState(DrivetrainState drivetrainState) {
        this.chassisSpeeds = drivetrainState.getChassisSpeeds();
        this.swerveModulePositions = drivetrainState.getModulePositions();
        this.robotRotation2d = drivetrainState.getRotation2d();
    }

    /**
     * Adds an AprilTag vision tracked translation3d WITHOUT timestamp.
     */ 
    public void addVisionPose2dUpdate(Pose2d robotVisionPose2d) {
        this.robotVisionPose2d = robotVisionPose2d;
    }

    public void updateRobotPose(Pose2d robotPose) {
        this.robotPose = robotPose;
    }

    public void updateNoteDetected(boolean noteDetected) {
        this.noteDetected = noteDetected;
    }

    /*
     *  Add Superstructure State
     */
    public void addSuperstructureState(SuperstructureState state) {
        this.superstructureState = state;
    }

    public SuperstructureState getSuperstructureState() {
        return superstructureState;
    }

    /**
     * Reset the RobotState's Initial Pose2d and set the NavX Offset. 
     * NavX offset is set when the robot has an inital rotation not facing where you want 0 (forwards) to be.
     */
    public void resetInitialPose(Pose2d initialStartingPose) {
        navxOffset = new Rotation2d();
        navxOffset = initialStartingPose.getRotation();
        initialPose = initialStartingPose;
    }

    /**
     * Returns the latest AprilTag vision detection robot translation in Translation2d
     * 
     * @return Translation2d
     */
    public Pose2d getVisionPose2d() {
        return robotVisionPose2d;
    }

    /**
     * Returns the latest AprilTag Vision detection time. This is when (on the raspberry pi) the Translation3d was last updated.
     * 
     * @return double
     */
    public double getVisionDetectionTime() {
        if(detectionTime == 0.0){
            return Timer.getFPGATimestamp();
        }
        return detectionTime;
    }

    /**
     * Returns the Rotation2d of the robot, accounting for an offset that was set when intialized. 
     * 
     * @return Rotation2d
     */
    public Rotation2d getRotation2d() {
        return robotRotation2d.rotateBy(navxOffset);
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

    public boolean getNoteDetected() {
        return noteDetected;
    }



    /**
     * Returns the initial Pose2d of the robot since last reset.
     * 
     * @return Pose2d
     */
    public Pose2d getInitialPose() {
        return initialPose;
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
        Dashboard.getInstance().putData("Rotation Degrees", robotRotation2d.getDegrees());
        Dashboard.getInstance().putData("Robot Position X Inches: ", Units.inchesToMeters(robotPose.getX()));
        Dashboard.getInstance().putData("Robot Position Y Inches: ", Units.inchesToMeters(robotPose.getY()));
        Dashboard.getInstance().putData("VISION Robot Position X Inches: ", Units.inchesToMeters(robotVisionPose2d.getX()));
        Dashboard.getInstance().putData("VISION Robot Position Y Inches: ", Units.inchesToMeters(robotVisionPose2d.getY()));
        Dashboard.getInstance().putData("Vision Rotational Value Degrees: ", robotVisionPose2d.getRotation().getDegrees());
    }   
}
