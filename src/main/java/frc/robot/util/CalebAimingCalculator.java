package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.states.ShamperAndRotationState;

public class CalebAimingCalculator {

    public static ShamperAndRotationState calculateShamperAngle(Pose2d pose, Translation2d speakerLocation, ChassisSpeeds chassisSpeeds) { 
        double angle;
        double horizontalVelocityNeeded;
        double verticalVelocityNeeded;
        double timeUntilTrajectoryTargetHeight;
        double predictedNoteVelocity;
        double noteTrajectoryAngle;

        double shotNoteVelocity = Constants.FieldAndRobot.NOTE_SPEED_IN_METERS_PER_SECOND;

        double xDistanceToSpeaker = pose.getX() - speakerLocation.getX();
        double yDistanceToSpeaker = pose.getY() - speakerLocation.getY();
        double distanceToSpeaker = Math.sqrt(Math.pow(xDistanceToSpeaker, 2) + Math.pow(yDistanceToSpeaker, 2));
        double angleToSpeaker = Math.atan(yDistanceToSpeaker / xDistanceToSpeaker);
        
        double robotXVelocity = chassisSpeeds.vxMetersPerSecond;
        double robotYVelocity = chassisSpeeds.vyMetersPerSecond;
        
        double robotCombinedVelocity = Math.sqrt(Math.pow(robotXVelocity, 2) + Math.pow(robotYVelocity, 2));
        robotCombinedVelocity = Math.sqrt(Math.pow(robotCombinedVelocity, 2) + Math.pow(shotNoteVelocity, 2));

        noteTrajectoryAngle = Math.acos(robotXVelocity/robotCombinedVelocity);
        System.out.println(Math.toDegrees(noteTrajectoryAngle));
        
        double differenceInAngle = noteTrajectoryAngle - angleToSpeaker;

        double targetHeight = Constants.FieldAndRobot.SPEAKER_TARGET_HEIGHT_OFF_GROUND_IN_METERS - Constants.Shamper.SPEAKER_TARGET_VERTICAL_OFFSET_IN_METERS;

        //calculate velocities
        verticalVelocityNeeded = Math.sqrt(2 * Constants.FieldAndRobot.GRAVITY_IN_METERS_PER_SECOND_SQUARED * targetHeight);
        timeUntilTrajectoryTargetHeight = (verticalVelocityNeeded / Constants.FieldAndRobot.GRAVITY_IN_METERS_PER_SECOND_SQUARED);
        System.out.println("vvn: " +verticalVelocityNeeded);
        
        predictedNoteVelocity = robotCombinedVelocity * Math.cos(differenceInAngle);
        System.out.println("predvelo: "+predictedNoteVelocity);

        horizontalVelocityNeeded = (distanceToSpeaker / timeUntilTrajectoryTargetHeight) - predictedNoteVelocity;
        System.out.println("hvn: "+horizontalVelocityNeeded);
        
        //calculate angle
        angle = Math.toDegrees(Math.atan(verticalVelocityNeeded / horizontalVelocityNeeded));
        
        System.out.println("FINAL SHAMPER ANGLE: " + angle);

        System.out.println("FINAL ROBOT ANGLE: " + Math.toDegrees(differenceInAngle));
        return new ShamperAndRotationState(angle, differenceInAngle);
    }
}