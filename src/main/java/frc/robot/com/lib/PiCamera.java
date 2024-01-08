package com.team2052.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import frc.robot.io.Dashboard;

public class PiCamera {
    private String cameraName;
    private Translation3d cameraTranslation2dMeters;
    private Transform3d cameraTransform3dOffsetsMeters;
    private double visionYaw;
    private Pose2d robotVisionPose2d;

    private DoubleArraySubscriber raspberryPiCameraPoseSubscriber;
    private double[] cameraPoseMeters = new double[0];
    private BooleanSubscriber raspberryPiHasValidTagReadingSubscriber;
    private boolean hasValidTagReading = false;

    public PiCamera(
        String cameraName,
        Transform3d cameraTransform3dOffsetsMeters
    ) {
        this.cameraName = cameraName;
        this.cameraTransform3dOffsetsMeters = cameraTransform3dOffsetsMeters;
        
        raspberryPiCameraPoseSubscriber = Dashboard.getInstance().getRaspberryPiCameraPoseMeters(cameraName).subscribe(cameraPoseMeters);
        raspberryPiHasValidTagReadingSubscriber = Dashboard.getInstance().getRaspberryPiValidReadingState(cameraName).subscribe(hasValidTagReading);
    }

    public Pose2d getEstimatedPosition(){
      cameraTranslation2dMeters = new Translation3d(raspberryPiCameraPoseSubscriber.get()[0], raspberryPiCameraPoseSubscriber.get()[1], raspberryPiCameraPoseSubscriber.get()[2]);
      visionYaw = raspberryPiCameraPoseSubscriber.get()[3];
      robotVisionPose2d = new Pose2d(
        cameraTranslation2dMeters.plus(cameraTransform3dOffsetsMeters.getTranslation()).toTranslation2d(), 
        new Rotation2d(Units.degreesToRadians(visionYaw))
      );

      return robotVisionPose2d;
    }

    public String getName(){
        return cameraName;
    }

    public boolean isValid(){
        return raspberryPiHasValidTagReadingSubscriber.get();
    }

    public void debug(){
      System.out.println(visionYaw);
    }
}
