package frc.robot.util;

import frc.robot.Constants;
import frc.robot.states.RobotState;
import frc.robot.states.ShamperAndRotationState;

public class AimingCalculator {
    
    public static ShamperAndRotationState calculate() {
        double robotAngle;
        double shamperAngle;

        double targetHeight = (Constants.FieldAndRobot.SPEAKER_TARGET_HEIGHT_OFF_GROUND_IN_METERS 
        - Constants.FieldAndRobot.SHAMPER_HEIGHT_IN_METERS) 
        + Constants.FieldAndRobot.SPEAKER_TARGET_HEIGHT_OFF_GROUND_IN_METERS;

        double robotXVelocity = RobotState.getInstance().getChassisSpeeds().vxMetersPerSecond;
        double robotYVelocity = RobotState.getInstance().getChassisSpeeds().vyMetersPerSecond;

        double xDistanceToSpeaker = ((RobotState.getInstance().isRedAlliance())
                ? Constants.FieldAndRobot.RED_SPEAKER_LOCATION.getX()
                        + Constants.FieldAndRobot.RED_SPEAKER_TARGET_X_OFFSET_IN_METERS
                : Constants.FieldAndRobot.BLUE_SPEAKER_LOCATION.getX()
                        + Constants.FieldAndRobot.BLUE_SPEAKER_TARGET_X_OFFSET_IN_METERS)
                - RobotState.getInstance().getRobotPose().getX();

        double yDistanceToSpeaker = ((RobotState.getInstance().isRedAlliance())
                ? Constants.FieldAndRobot.RED_SPEAKER_LOCATION.getY()
                        + Constants.FieldAndRobot.RED_SPEAKER_TARGET_Y_OFFSET_IN_METERS
                : Constants.FieldAndRobot.BLUE_SPEAKER_LOCATION.getY()
                        + Constants.FieldAndRobot.BLUE_SPEAKER_TARGET_Y_OFFSET_IN_METERS)
                - RobotState.getInstance().getRobotPose().getY();

        double fullDistanceToSpeaker = RobotState.getInstance().getRobotPose().getTranslation()
                .getDistance((RobotState.getInstance().isRedAlliance()) ? Constants.FieldAndRobot.RED_SPEAKER_LOCATION
                        : Constants.FieldAndRobot.BLUE_SPEAKER_LOCATION);

        double robotVelocityAngle;
        if (robotXVelocity == 0) {
            robotVelocityAngle = (robotYVelocity <= 0) ? 
            Math.toRadians(-90) : (robotYVelocity >= 0) ? 
            Math.toRadians(90) : 0;
        } else {
            robotVelocityAngle = Math.atan(robotYVelocity / robotXVelocity);
        }

        double angleToSpeaker;

        if (xDistanceToSpeaker == 0) {
            angleToSpeaker = (yDistanceToSpeaker <= 0) ? Math.toRadians(90) : (yDistanceToSpeaker >= 0) ? Math.toRadians(-90) : 0;
        } else {
            angleToSpeaker = Math.atan(yDistanceToSpeaker / xDistanceToSpeaker);
        }

        double differenceInAngle = robotVelocityAngle - angleToSpeaker;

        double robotCombinedVelocity = Math.sqrt(Math.pow(robotXVelocity, 2) + Math.pow(robotYVelocity, 2));

        double verticalVelocityNeeded = Math.sqrt(2 * Constants.FieldAndRobot.GRAVITY_IN_METERS_PER_SECOND_SQUARED * targetHeight);
        double timeUntilTrajectoryTargetHeight = (verticalVelocityNeeded / Constants.FieldAndRobot.GRAVITY_IN_METERS_PER_SECOND_SQUARED);

        double predictedNoteVelocity = 2 * robotCombinedVelocity * Math.cos(differenceInAngle);

        double horizontalVelocityNeeded = (fullDistanceToSpeaker / timeUntilTrajectoryTargetHeight) - predictedNoteVelocity;

        shamperAngle = Math.atan(verticalVelocityNeeded / horizontalVelocityNeeded);

        double noteSpeedAfterLaunch = Constants.FieldAndRobot.NOTE_SPEED_IN_METERS_PER_SECOND * Math.acos(shamperAngle);
        
        robotAngle = Math.toDegrees(angleToSpeaker - ((robotCombinedVelocity * differenceInAngle) / noteSpeedAfterLaunch)) + Constants.Drivetrain.ROBOT_AIMING_ROTATION_OFFSET_IN_DEGREES;

        return new ShamperAndRotationState(shamperAngle, robotAngle);
    }
}
