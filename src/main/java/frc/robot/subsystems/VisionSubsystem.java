// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
  private static PhotonCamera noteCamera;
  private static boolean hasTarget;
  private static List<PhotonTrackedTarget> allTargets;  
  private static PhotonTrackedTarget target;
  private static Pose2d targetPose;
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    noteCamera = new PhotonCamera("Note Detection Camera Team 2052");
  }

  @Override
  public void periodic() {
    var cameraResult = noteCamera.getLatestResult();
    hasTarget = cameraResult.hasTargets();
    if (hasTarget) {
      allTargets = cameraResult.getTargets();
      target = cameraResult.getBestTarget();
      targetPose = new Pose2d(
        new Translation2d(
          target.getAlternateCameraToTarget().getX() + Constants.Vision.NOTE_DETECTION_CAMERA_X_OFFSET, 
          target.getAlternateCameraToTarget().getY() + Constants.Vision.NOTE_DETECTION_CAMERA_Y_OFFSET), 
          new Rotation2d(Math.cos(target.getAlternateCameraToTarget().getRotation().getAngle() + Constants.Vision.NOTE_DETECTION_CAMERA_ROTATION_OFFSET), Math.sin(target.getAlternateCameraToTarget().getRotation().getAngle() + Constants.Vision.NOTE_DETECTION_CAMERA_ROTATION_OFFSET)));
    }
    // This method will be called once per scheduler run
  }

  public boolean hasTarget() {
    return hasTarget;
  }

  public Pose2d getBestNotePose() {
    return targetPose;
  }
}