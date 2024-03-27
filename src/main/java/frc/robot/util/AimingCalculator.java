package frc.robot.util;

import java.util.Optional;
import java.util.Vector;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.AprilTagSubsystem;

public class AimingCalculator {

    public static double calculateStill(Pose2d robotPose) {
        Rotation2d angleToSpeaker = new Rotation2d();
        double xDistanceToSpeaker = 0;
        double yDistanceToSpeaker = 0;

        if(!(RobotState.getInstance().isRedAlliance())){
                xDistanceToSpeaker = Math.abs(Constants.FieldAndRobot.BLUE_SPEAKER_LOCATION.getX() + Constants.FieldAndRobot.BLUE_SPEAKER_TARGET_X_OFFSET_IN_METERS - robotPose.getX());
                yDistanceToSpeaker = Math.abs(Constants.FieldAndRobot.BLUE_SPEAKER_LOCATION.getY() + Constants.FieldAndRobot.BLUE_SPEAKER_TARGET_Y_OFFSET_IN_METERS - robotPose.getY());

                angleToSpeaker = new Rotation2d(Math.atan(yDistanceToSpeaker / xDistanceToSpeaker));

                //angleToSpeaker = Rotation2d.fromDegrees(Math.copySign(angleToSpeaker.getDegrees(), Constants.FieldAndRobot.BLUE_SPEAKER_LOCATION.getY() - robotPose.getY())); 
        }

        //Logger.recordOutput("robot to speaker", new Pose2d(robotPose.getTranslation(), angleToSpeaker.rotateBy(new Rotation2d(Math.PI)).unaryMinus()));
        //Logger.recordOutput("speaker location", new Pose2d(Constants.FieldAndRobot.BLUE_SPEAKER_LOCATION, new Rotation2d(0)));

        double robotTargetAngle = angleToSpeaker.rotateBy(new Rotation2d(Math.PI)).unaryMinus().getDegrees();
        return robotTargetAngle;
    }

    public static double calculateAngle(Pose2d robotPose){
        double xDistance = 0;
        double yDistance = 0;
        double speakerToRobotDegrees = 0;
        double angleToSpeakerFieldRelativeDegrees = 0;

        // blue alliance
        if(!RobotState.getInstance().isRedAlliance()) {
            Translation2d speakerLocation = Constants.FieldAndRobot.BLUE_SPEAKER_LOCATION;
            xDistance = speakerLocation.getX() - robotPose.getX();
            yDistance = speakerLocation.getY() - robotPose.getY();
            // System.out.println(robotPose.getY());
            speakerToRobotDegrees = Units.radiansToDegrees(Math.atan(Math.abs(yDistance) / Math.abs(xDistance)));
            Logger.recordOutput("Aiming Calculator STR", speakerToRobotDegrees);
            // to the right of speaker
            if(robotPose.getY() < speakerLocation.getY()) {
                // System.out.println("RIGHT SIDE");
                angleToSpeakerFieldRelativeDegrees = 180 + speakerToRobotDegrees;
            }

            // to the left of the speaker
            if(robotPose.getY() > speakerLocation.getY()) {
                // System.out.println("LEFT SIDE");
                angleToSpeakerFieldRelativeDegrees = 175 - speakerToRobotDegrees;
                
                //angleToSpeakerFieldRelativeDegrees2 = Math.abs(new Rotation2d(Math.toRadians(90 + angleToSpeakerRobotRelativeDegrees)).unaryMinus().getDegrees());
            }
        } else { // red alliance
            Translation2d speakerLocation = Constants.FieldAndRobot.RED_SPEAKER_LOCATION;
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

    public static double angleToPoint(Translation2d goalPoint, Pose2d robotPose, ChassisSpeeds robotChassisSpeeds) {
        Translation2d robotPos = robotPose.getTranslation();
        double distanceToTarget = goalPoint.getDistance(robotPos);
        Rotation2d angleToTarget = goalPoint.minus(robotPos).getAngle();

        Translation2d robotVelo = new Translation2d(robotChassisSpeeds.vxMetersPerSecond, robotChassisSpeeds.vyMetersPerSecond).rotateBy(angleToTarget);

        double flyTime = 0.0;
        double uncorrectedShot = flyTime * robotVelo.getY();

        double correction = -Math.atan2(uncorrectedShot, distanceToTarget);

        double setpointAngle = MathUtil.inputModulus(angleToTarget.getRadians() + correction, -Math.PI, Math.PI);
        Logger.recordOutput("setpoint", setpointAngle);
        
        return setpointAngle;
    }

    public static double calculateRobotAngleOneAprilTag() {
        Optional<Double> yaw = AprilTagSubsystem.getInstance().getYaw();
     
        if (!yaw.isEmpty()) {
            return yaw.get();
        } else {
            return 0;
        }
    }

    public static double calculateAngleToSpeaker(Pose2d robotPose) {
        if(!RobotState.getInstance().isRedAlliance()) { // blue alliance
            Translation2d speakerLocation = Constants.FieldAndRobot.BLUE_SPEAKER_LOCATION;
            double xDistance = Math.abs(speakerLocation.getX() - robotPose.getX());
            double yDistance = Math.abs(speakerLocation.getY() - robotPose.getY());

            double speakerToRobotDegrees = Units.radiansToDegrees(Math.atan(Math.abs(yDistance) / Math.abs(xDistance)));
            return speakerToRobotDegrees;
        } else { // red alliance
            Translation2d speakerLocation = Constants.FieldAndRobot.RED_SPEAKER_LOCATION;
            double xDistance = Math.abs(speakerLocation.getX() - robotPose.getX());
            double yDistance = Math.abs(speakerLocation.getY() - robotPose.getY());
            
            double speakerToRobotDegrees = Units.radiansToDegrees(Math.atan(Math.abs(yDistance) / Math.abs(xDistance)));
            return speakerToRobotDegrees;
        }
    }
    
    public static double calculateDistanceToAimPoint(Pose2d robotPose) {
        if(!RobotState.getInstance().isRedAlliance()) { // blue alliance
            Translation2d aimLocation = Constants.FieldAndRobot.BLUE_SPEAKER_LOCATION;

            if (calculateAngleToSpeaker(robotPose) > 30) { // if we are too far to the left or right aim for a better shot
                System.out.println("applying: " + calculateAngleToSpeaker(robotPose));
                if(aimLocation.getY() > robotPose.getY()) { //to the left of speaker (robot pov)
                    // since we are to the left, we want to offset our aim point to the right for a better shot
                    aimLocation = aimLocation.plus(new Translation2d(0, +0.2));
                } else { //to the right of speaker
                    // since we are to the right, we want to offset our aim point to the left for a better shot
                    aimLocation = aimLocation.plus(new Translation2d(0, -0.2));
                }
            }

            System.out.println("Angle: " + calculateAngleToSpeaker(robotPose) + " Speaker Location: " + Constants.FieldAndRobot.BLUE_SPEAKER_LOCATION.getY() + " Aim Location: " + aimLocation.getY());

            double xDistance = Math.abs(aimLocation.getX() - robotPose.getX());
            double yDistance = Math.abs(aimLocation.getY() - robotPose.getY());

            return Math.hypot(xDistance, yDistance);
        } else { // red alliance
            Translation2d aimLocation = Constants.FieldAndRobot.RED_SPEAKER_LOCATION;

            if (calculateAngleToSpeaker(robotPose) > 30) { // if we are too far to the left or right aim for a better shot
                if(aimLocation.getY() < robotPose.getY()) { //to the left of speaker (robot pov)
                    // since we are to the left, we want to offset our aim point to the right for a better shot
                    aimLocation.plus(new Translation2d(0, -0.2));
                } else { //to the right of speaker
                    // since we are to the right, we want to offset our aim point to the left for a better shot
                    aimLocation.plus(new Translation2d(0, +0.2));
                }
            }
            double xDistance = Math.abs(aimLocation.getX() - robotPose.getX());
            double yDistance = Math.abs(aimLocation.getY() - robotPose.getY());
            
            return Math.hypot(xDistance, yDistance);
        }
    }
}
