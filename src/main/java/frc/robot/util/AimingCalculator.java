package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.FieldAndRobot;
import frc.robot.Constants.Shamper;
import frc.robot.states.RobotState;

public class AimingCalculator {

    private Translation2d speakerLocation;
    private double targetHeight;
    private double distanceToSpeaker;
    private Pose2d pose;
    private Translation2d location;

    private double xDistanceToSpeaker;
    private double yDistanceToSpeaker;

    private double angleToSpeaker;

    private double robotXVelocity;
    private double robotYVelocity;
    private double robotCombinedVelocity;

    private RobotState robotState = RobotState.getInstance();

    private double velocityAngle;
    private double differenceInAngle;

    private double noteSpeedAfterLaunch;
    
    
    public AimingCalculator() {

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            speakerLocation = (alliance.get() == DriverStation.Alliance.Red) ? 
            Constants.FieldAndRobot.RED_SPEAKER_LOCATION : 
            Constants.FieldAndRobot.BLUE_SPEAKER_LOCATION;
        } else {
            speakerLocation = Constants.FieldAndRobot.RED_SPEAKER_LOCATION;
        }

        targetHeight = (Constants.FieldAndRobot.SPEAKER_TARGET_HIGHT_OFF_GROUND_IN_METERS - Constants.FieldAndRobot.SHAMPER_HIGHT_IN_METERS) + Constants.Shamper.SPEAKER_TARGET_VERTICAL_OFFSET_IN_METERS;
        speakerLocation.plus(new Translation2d(Constants.Shamper.SPEAKER_TARGET_X_OFFSET_IN_METERS, Constants.Shamper.SPEAKER_TARGET_Y_OFFSET_IN_METERS));
    }

    // TODO: fix this mess

    public void updateInformation() {
        pose = robotState.getRobotPose();
        location = pose.getTranslation();
        distanceToSpeaker = location.getDistance(speakerLocation);
        xDistanceToSpeaker = speakerLocation.getX() - location.getX();
        yDistanceToSpeaker = speakerLocation.getY() - location.getY();
        angleToSpeaker = Math.atan(yDistanceToSpeaker / xDistanceToSpeaker);

        robotXVelocity = robotState.getChassisSpeeds().vxMetersPerSecond;
        robotYVelocity = robotState.getChassisSpeeds().vyMetersPerSecond;

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

    public static double getStationaryTargetShamperAngle() { 
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

    public static double getMovingTargetShamperAngle() {
        double angle;
        double horizontalVelocityNeeded;
        double verticalVelocityNeeded;
        double timeUntilTrajectoryTargetHeight;
        double predictedNoteVelocity;

        verticalVelocityNeeded = Math.sqrt(2 * Constants.FieldAndRobot.GRAVITY_IN_METERS_PER_SECOND_SQUARED * targetHeight);
        timeUntilTrajectoryTargetHeight = (verticalVelocityNeeded / Constants.FieldAndRobot.GRAVITY_IN_METERS_PER_SECOND_SQUARED);

        predictedNoteVelocity = 2 * robotCombinedVelocity * Math.cos(differenceInAngle);

        horizontalVelocityNeeded = (distanceToSpeaker / timeUntilTrajectoryTargetHeight) - predictedNoteVelocity;

        angle = Math.toDegrees(Math.asin(verticalVelocityNeeded / horizontalVelocityNeeded));
        return angle;
    }

    public static double getMovingTargetRobotAngle() {
        double angle;
        angle = Math.toDegrees(angleToSpeaker - ((robotCombinedVelocity * differenceInAngle) / noteSpeedAfterLaunch));
        return angle;
    }
}