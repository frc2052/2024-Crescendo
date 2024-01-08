// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class Dashboard {
        public static final String DRIVE_MODE_KEY = "Drive Mode";
    }

    public static class Drivetrain {
        // Left-to-right distance between drivetrain wheels
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0;
        // Front-to-back distance between drivetrain wheels
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0;

        // TODO: Add drivetrain constants
        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 0;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 0;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 0;
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET_RADIANS = 4.335 - (Math.PI / 2);

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 0;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 0;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 0;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET_RADIANS = 4.858 - (Math.PI / 2);

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 0;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 0;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 0;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET_RADIANS = 1.612 - (Math.PI / 2);

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 0;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 0;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 0;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET_RADIANS = 1.179 - (Math.PI / 2);

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0)
        );
    }

    public static final class PiCamera1 {
        // TODO: Add PiCamera offsets
        public static final double X_OFFSET_INCHES = 0;
        public static final double Y_OFFSET_INCHES = 0;
        public static final double Z_OFFSET_INCHES = 0;

        public static final double THETA_X_OFFSET_DEGREES = 0.0; // roll
        public static final double THETA_Y_OFFSET_DEGREES = 0.0; // pitch
        public static final double THETA_Z_OFFSET_DEGREES = 0.0; // yaw

        public static final Transform3d PI_CAMERA_POSITION_METERS = new Transform3d(
            new Translation3d(Units.inchesToMeters(X_OFFSET_INCHES), Units.inchesToMeters(Y_OFFSET_INCHES), Units.inchesToMeters(Z_OFFSET_INCHES)), 
            new Rotation3d(Units.degreesToRadians(THETA_X_OFFSET_DEGREES), Units.degreesToRadians(THETA_Y_OFFSET_DEGREES), Units.degreesToRadians(THETA_Z_OFFSET_DEGREES))
        );
    }

    public static final class AprilTagLocations {
        /*
         * XYZ Origin is bottom left corner of field, 
         * x = 0 is aligned with Blue Alliance Station diamond plate 
         * y = 0 is aligned with the side border polycarb on the Scoring Table side of the field
         * z = 0 is the carpet
         * 
         * +z is up in the air from the carpet
         * +x is towards red alliance stations
         * +y from field border towards speakers
         * 
         * rotation is along the Z-axis
         * 0 degrees faces red alliance station, 90 degrees faces non scoring table side, and 180 degrees faces blue alliance station
         * 
         * ALL DISTANCES ARE TO THE CENTER OF THE TAG
         */

        public static final Translation3d TAG1_TRANSLATION3D = new Translation3d(593.68, 9.68, 53.38);
        public static final double TAG1_ROTATION = 120;
        public static final Translation3d TAG2_TRANSLATION3D = new Translation3d(637.21, 34.79, 53.38);
        public static final double TAG2_ROTATION = 120;
        public static final Translation3d TAG3_TRANSLATION3D = new Translation3d(652.73, 196.17, 57.13);
        public static final double TAG3_ROTATION = 180;
        public static final Translation3d TAG4_TRANSLATION3D = new Translation3d(652.730, 218.42, 57.13);
        public static final double TAG4_ROTATION = 180;
        public static final Translation3d TAG5_TRANSLATION3D = new Translation3d(578.77, 323.00, 53.38);
        public static final double TAG5_ROTATION = 270;
        public static final Translation3d TAG6_TRANSLATION3D = new Translation3d(72.50, 323.00, 53.38);
        public static final double TAG6_ROTATION = 270;
        public static final Translation3d TAG7_TRANSLATION3D = new Translation3d(-1.50, 218.42, 57.13);
        public static final double TAG7_ROTATION = 0;
        public static final Translation3d TAG8_TRANSLATION3D = new Translation3d(-1.50, 196.17, 57.13);
        public static final double TAG8_ROTATION = 0;
        public static final Translation3d TAG9_TRANSLATION3D = new Translation3d(14.02, 34.79, 53.38);
        public static final double TAG9_ROTATION = 60;
        public static final Translation3d TAG10_TRANSLATION3D = new Translation3d(57.54, 9.68, 53.38);
        public static final double TAG10_ROTATION = 60;
        public static final Translation3d TAG11_TRANSLATION3D = new Translation3d(468.69, 146.19, 52.00);
        public static final double TAG11_ROTATION = 300;
        public static final Translation3d TAG12_TRANSLATION3D = new Translation3d(468.69, 177.10, 52.00);
        public static final double TAG12_ROTATION = 60;
        public static final Translation3d TAG13_TRANSLATION3D = new Translation3d(441.74, 161.62, 52.00);
        public static final double TAG13_ROTATION = 180;
        public static final Translation3d TAG14_TRANSLATION3D = new Translation3d(209.48, 161.62, 52.00);
        public static final double TAG14_ROTATION = 0;
        public static final Translation3d TAG15_TRANSLATION3D = new Translation3d(182.73, 177.10, 52.00);
        public static final double TAG15_ROTATION = 120;
        public static final Translation3d TAG16_TRANSLATION3D = new Translation3d(182.73, 146.19, 52.00);
        public static final double TAG16_ROTATION = 240;
        AprilTagFieldLayout apriltagLayout = new AprilTagFieldLayout(null)
    }


}
