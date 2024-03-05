package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.states.ShamperAndRotationState;

public class AimingCalculator {
    
//     public static ShamperAndRotationState calculate() {
//         double robotAngle;
//         double shamperAngle;

//         double targetHeight = (Constants.FieldAndRobot.SPEAKER_TARGET_HEIGHT_OFF_GROUND_IN_METERS 
//         - Constants.FieldAndRobot.SHAMPER_HEIGHT_IN_METERS);

//         double robotXVelocity = RobotState.getInstance().getChassisSpeeds().vxMetersPerSecond;
//         double robotYVelocity = RobotState.getInstance().getChassisSpeeds().vyMetersPerSecond;

//         double xDistanceToSpeaker = ((RobotState.getInstance().isRedAlliance())
//                 ? Constants.FieldAndRobot.RED_SPEAKER_LOCATION.getX()
//                         + Constants.FieldAndRobot.RED_SPEAKER_TARGET_X_OFFSET_IN_METERS
//                 : Constants.FieldAndRobot.BLUE_SPEAKER_LOCATION.getX()
//                         + Constants.FieldAndRobot.BLUE_SPEAKER_TARGET_X_OFFSET_IN_METERS)
//                 // - RobotState.getInstance().getRobotPose().getX();
//                 - Units.inchesToMeters(114);

//         double yDistanceToSpeaker = ((RobotState.getInstance().isRedAlliance())
//                 ? Constants.FieldAndRobot.RED_SPEAKER_LOCATION.getY()
//                         + Constants.FieldAndRobot.RED_SPEAKER_TARGET_Y_OFFSET_IN_METERS
//                 : Constants.FieldAndRobot.BLUE_SPEAKER_LOCATION.getY()
//                         + Constants.FieldAndRobot.BLUE_SPEAKER_TARGET_Y_OFFSET_IN_METERS)
//                 // - RobotState.getInstance().getRobotPose().getY();
//                 - Units.inchesToMeters(0);

//         double fullDistanceToSpeaker = RobotState.getInstance().getRobotPose().getTranslation()
//                 .getDistance((RobotState.getInstance().isRedAlliance()) ? Constants.FieldAndRobot.RED_SPEAKER_LOCATION
//                         : Constants.FieldAndRobot.BLUE_SPEAKER_LOCATION);

//         double robotVelocityAngle;
//         if (robotXVelocity == 0) {
//             robotVelocityAngle = (robotYVelocity <= 0) ? 
//             Math.toRadians(-90) : (robotYVelocity >= 0) ? 
//             Math.toRadians(90) : 0;
//         } else {
//             robotVelocityAngle = Math.atan(robotYVelocity / robotXVelocity);
//         }

//         double angleToSpeaker;

//         if (xDistanceToSpeaker == 0) {
//             angleToSpeaker = (yDistanceToSpeaker <= 0) ? Math.toRadians(90) : (yDistanceToSpeaker >= 0) ? Math.toRadians(-90) : 0;
//         } else {
//             angleToSpeaker = Math.atan(yDistanceToSpeaker / xDistanceToSpeaker);
//         }

//         double differenceInAngle = robotVelocityAngle - angleToSpeaker;

//         double robotCombinedVelocity = Math.sqrt(Math.pow(robotXVelocity, 2) + Math.pow(robotYVelocity, 2));

//         double verticalVelocityNeeded = Math.sqrt(2 * Constants.FieldAndRobot.GRAVITY_IN_METERS_PER_SECOND_SQUARED * targetHeight);
//         double timeUntilTrajectoryTargetHeight = (verticalVelocityNeeded / Constants.FieldAndRobot.GRAVITY_IN_METERS_PER_SECOND_SQUARED);

//         double predictedNoteVelocity = 2 * robotCombinedVelocity * Math.cos(differenceInAngle);

//         double horizontalVelocityNeeded = (fullDistanceToSpeaker / timeUntilTrajectoryTargetHeight) - predictedNoteVelocity;

//         shamperAngle = Math.atan(verticalVelocityNeeded / horizontalVelocityNeeded);

//         double noteSpeedAfterLaunch = Constants.FieldAndRobot.NOTE_SPEED_IN_METERS_PER_SECOND * Math.acos(shamperAngle);
        
//         robotAngle = Math.toDegrees(angleToSpeaker - ((robotCombinedVelocity * differenceInAngle) / noteSpeedAfterLaunch)) + Constants.Drivetrain.ROBOT_AIMING_ROTATION_OFFSET_IN_DEGREES;

//         return new ShamperAndRotationState(shamperAngle, robotAngle);
//     }

    // public static double calculateStill(Pose2d robotPose) {
    //     Rotation2d angleToSpeaker = new Rotation2d();
    //     double xDistanceToSpeaker = 0;
    //     double yDistanceToSpeaker = 0;

    //     if(!(RobotState.getInstance().isRedAlliance())){
    //             xDistanceToSpeaker = Math.abs(Constants.FieldAndRobot.BLUE_SPEAKER_LOCATION.getX() + Constants.FieldAndRobot.BLUE_SPEAKER_TARGET_X_OFFSET_IN_METERS - robotPose.getX());
    //             yDistanceToSpeaker = Math.abs(Constants.FieldAndRobot.BLUE_SPEAKER_LOCATION.getY() + Constants.FieldAndRobot.BLUE_SPEAKER_TARGET_Y_OFFSET_IN_METERS - robotPose.getY());

    //             angleToSpeaker = new Rotation2d(Math.atan(yDistanceToSpeaker / xDistanceToSpeaker));

    //             //angleToSpeaker = Rotation2d.fromDegrees(Math.copySign(angleToSpeaker.getDegrees(), Constants.FieldAndRobot.BLUE_SPEAKER_LOCATION.getY() - robotPose.getY())); 
    //     }

    //     //Logger.recordOutput("robot to speaker", new Pose2d(robotPose.getTranslation(), angleToSpeaker.rotateBy(new Rotation2d(Math.PI)).unaryMinus()));
    //     //Logger.recordOutput("speaker location", new Pose2d(Constants.FieldAndRobot.BLUE_SPEAKER_LOCATION, new Rotation2d(0)));

    //     double robotTargetAngle = angleToSpeaker.rotateBy(new Rotation2d(Math.PI)).unaryMinus().getDegrees();
    //     return robotTargetAngle;
    // }

    public static double calculateAngle(Pose2d robotPose){
        double xDistance = 0;
        double yDistance = 0;
        double speakerToRobotDegrees = 0;
        double angleToSpeakerFieldRelativeDegrees = 0;

        // blue alliance
        if(!RobotState.getInstance().isRedAlliance()) {
            Translation2d speakerLocation = (Constants.FieldAndRobot.BLUE_SPEAKER_LOCATION.plus(
                new Translation2d(Constants.FieldAndRobot.BLUE_SPEAKER_TARGET_X_OFFSET_IN_METERS, Constants.FieldAndRobot.BLUE_SPEAKER_TARGET_Y_OFFSET_IN_METERS)));
            xDistance = speakerLocation.getX() - robotPose.getX();
            yDistance = speakerLocation.getY() - robotPose.getY();
            // System.out.println(robotPose.getY());
            speakerToRobotDegrees = Units.radiansToDegrees(Math.atan(Math.abs(yDistance) / Math.abs(xDistance)));
            Logger.recordOutput("Aiming Calculator STR", speakerToRobotDegrees);
            // to the right of speaker
            if(robotPose.getY() < speakerLocation.getY()) {
                // System.out.println("RIGHT SIDE");
                angleToSpeakerFieldRelativeDegrees = 180 - speakerToRobotDegrees;
            }

            // to the left of the speaker
            if(robotPose.getY() > speakerLocation.getY()) {
                // System.out.println("LEFT SIDE");
                angleToSpeakerFieldRelativeDegrees = 175 - speakerToRobotDegrees;
                
                //angleToSpeakerFieldRelativeDegrees2 = Math.abs(new Rotation2d(Math.toRadians(90 + angleToSpeakerRobotRelativeDegrees)).unaryMinus().getDegrees());
            }
        } else { // red alliance
            Translation2d speakerLocation = (Constants.FieldAndRobot.RED_SPEAKER_LOCATION.plus(
                new Translation2d(Constants.FieldAndRobot.RED_SPEAKER_TARGET_X_OFFSET_IN_METERS, Constants.FieldAndRobot.RED_SPEAKER_TARGET_Y_OFFSET_IN_METERS)));
            xDistance = speakerLocation.getX() - robotPose.getX();
            yDistance = speakerLocation.getY() - robotPose.getY();

            speakerToRobotDegrees = Units.radiansToDegrees(Math.atan(Math.abs(yDistance) / Math.abs(xDistance)));

            // to the right of speaker
            if(robotPose.getY() < speakerLocation.getY()) {
                angleToSpeakerFieldRelativeDegrees = 180 + speakerToRobotDegrees;
            }

            // to the left of the speaker
            if(robotPose.getY() > speakerLocation.getY()) {
                angleToSpeakerFieldRelativeDegrees = 172 - speakerToRobotDegrees;
                
                //angleToSpeakerFieldRelativeDegrees2 = Math.abs(new Rotation2d(Math.toRadians(90 + angleToSpeakerRobotRelativeDegrees)).unaryMinus().getDegrees());
            }

        }
        Logger.recordOutput("ANGLE TO SPEAKER ROBOT RELATIVE", speakerToRobotDegrees);
        Logger.recordOutput("ANGLE TO SPEAKER FIELD RELATIVE", angleToSpeakerFieldRelativeDegrees);
        return MathUtil.inputModulus(Math.copySign(angleToSpeakerFieldRelativeDegrees, robotPose.getRotation().getDegrees()), 0, 360);
    }

    public static double calculateDistanceToSpeaker(Pose2d robotPose) {
        if(!RobotState.getInstance().isRedAlliance()) { // blue alliance
            Translation2d speakerLocation = Constants.FieldAndRobot.BLUE_SPEAKER_LOCATION.plus(
                new Translation2d(Constants.FieldAndRobot.BLUE_SPEAKER_TARGET_X_OFFSET_IN_METERS, Constants.FieldAndRobot.BLUE_SPEAKER_TARGET_Y_OFFSET_IN_METERS));
            double xDistance = Math.abs(speakerLocation.getX() - robotPose.getX());
            double yDistance = Math.abs(speakerLocation.getY() - robotPose.getY());

            return Math.hypot(xDistance, yDistance);
        } else { // red alliance
            Translation2d speakerLocation = Constants.FieldAndRobot.RED_SPEAKER_LOCATION.plus(
                new Translation2d(Constants.FieldAndRobot.RED_SPEAKER_TARGET_X_OFFSET_IN_METERS, Constants.FieldAndRobot.RED_SPEAKER_TARGET_Y_OFFSET_IN_METERS));
            double xDistance = Math.abs(speakerLocation.getX() - robotPose.getX());
            double yDistance = Math.abs(speakerLocation.getY() - robotPose.getY());
            
            return Math.hypot(xDistance, yDistance);
        }
    }
}
