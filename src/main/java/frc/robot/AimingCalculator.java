package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AimingCalculator {

    private static Translation2d speakerLocation;
    private static double targetHeight;
    private static double distanceToSpeaker;
    private static Pose2d pose;
    private static Translation2d location;

    private static double xDistanceToSpeaker;
    private static double yDistanceToSpeaker;

    private static double angleToSpeaker;

    private static double robotXVelocity;
    private static double robotYVelocity;
    private static double robotCombinedVelocity;

    private static DrivetrainSubsystem drivetrain;

    private static double velocityAngle;
    private static double differenceInAngle;

    private static double noteSpeedAfterLaunch;
    
    
    public void AimingCalculator() {

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            speakerLocation = (alliance.get() == DriverStation.Alliance.Red) ? 
            Constants.FieldAndRobot.RED_SPEAKER_LOCATION : 
            Constants.FieldAndRobot.BLUE_SPEAKER_LOCATION;
        } else {
            speakerLocation = Constants.FieldAndRobot.RED_SPEAKER_LOCATION;
        }

        targetHeight = (Constants.FieldAndRobot.SPEAKER_TARGET_HIGHT_OFF_GROUND_IN_METERS - Constants.FieldAndRobot.SHOOTER_HIGHT_IN_METERS) + Constants.VerticalShooter.SPEAKER_TARGET_VERTICAL_OFFSET_IN_METERS;
        speakerLocation.plus(new Translation2d(Constants.VerticalShooter.SPEAKER_TARGET_X_OFFSET_IN_METERS, Constants.VerticalShooter.SPEAKER_TARGET_Y_OFFSET_IN_METERS));
    }

    public static void updateInformation() {
        pose = RobotState.getInstance().getRobotPose();
        location = pose.getTranslation();
        distanceToSpeaker = location.getDistance(speakerLocation);
        xDistanceToSpeaker = speakerLocation.getX() - location.getX();
        yDistanceToSpeaker = speakerLocation.getY() - location.getY();
        angleToSpeaker = Math.atan(yDistanceToSpeaker / xDistanceToSpeaker);

        robotXVelocity = drivetrain.getSpeeds().vxMetersPerSecond;
        robotYVelocity = drivetrain.getSpeeds().vyMetersPerSecond;

        robotCombinedVelocity = Math.sqrt(Math.pow(robotXVelocity, 2) + Math.pow(robotYVelocity, 2));

        if (robotXVelocity == 0) {
            velocityAngle = (robotYVelocity <= 0) ? 
            Math.toRadians(-90) : (robotYVelocity >= 0) ? 
            Math.toRadians(90) : 0;
        } else {
            velocityAngle = Math.atan(robotYVelocity / robotXVelocity);
        }

        differenceInAngle = velocityAngle - angleToSpeaker;

        noteSpeedAfterLaunch = Constants.FieldAndRobot.NOTE_SPEED_IN_METERS_PER_SECOND * Math.acos(Math.toDegrees(getMovingTargetRobotAngle()));
    }

    public static double getStationaryTargetShooterAngle() { 
        double angle;
        double horizontalVelocityNeeded;
        double verticalVelocityNeeded;
        double timeUntilTrajectoryTargetHeight;
        
        //calculate velocities
        verticalVelocityNeeded = Math.sqrt(2 * Constants.FieldAndRobot.GRAVITY_IN_METERS_PER_SECOND_SQUARED * targetHeight);
        timeUntilTrajectoryTargetHeight = (verticalVelocityNeeded / Constants.FieldAndRobot.GRAVITY_IN_METERS_PER_SECOND_SQUARED);
        
        horizontalVelocityNeeded = distanceToSpeaker / timeUntilTrajectoryTargetHeight;
        
        //calculate angle
        angle = Math.toDegrees(Math.asin(verticalVelocityNeeded / horizontalVelocityNeeded));
        
        return angle;
    }

    public static double getStationaryTargetRobotAngle() { 
        return angleToSpeaker;
    }

    public static double getMovingTargetShooterAngle() {
        double angle;
        double horizontalVelocityNeeded;
        double verticalVelocityNeeded;
        double timeUntilTrajectoryTargetHeight;
        double predictedNoteVelocity;

        verticalVelocityNeeded = Math.sqrt(2 * Constants.FieldAndRobot.GRAVITY_IN_METERS_PER_SECOND_SQUARED * targetHeight);
        timeUntilTrajectoryTargetHeight = (verticalVelocityNeeded / Constants.FieldAndRobot.GRAVITY_IN_METERS_PER_SECOND_SQUARED);

        predictedNoteVelocity = 2 * robotCombinedVelocity * Math.cos(differenceInAngle);

        horizontalVelocityNeeded = (distanceToSpeaker / timeUntilTrajectoryTargetHeight) - predictedNoteVelocity;

        angle = Math.toDegrees(Math.asin(verticalVelocityNeeded / horizontalVelocityNeeded)) + Constants.Drivetrain.ROBOT_AIMING_ROTATION_OFFSET_IN_DEGREES;
        return angle;
    }

    public static double getMovingTargetRobotAngle() {
        double angle;
        angle = Math.toDegrees(angleToSpeaker - ((robotCombinedVelocity * differenceInAngle) / noteSpeedAfterLaunch)) + Constants.Drivetrain.ROBOT_AIMING_ROTATION_OFFSET_IN_DEGREES;
        return angle;
    }
}