// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.ShamperSubsystem;

public final class Constants {

    // TODO: Put in all the constants

    public static class MotorConstants {
        public static final double FALCON500_TICKS_PER_ROTATION = 2048;
    }

    public static class Climber{
        public static final int CLIMBER_MOTOR = 0;
        public static final int CLIMBER_EXTENSION_HEIGHT_TICKS = 0;
        public static final int CLIMBER_RETRACTION_HEIGHT_TICKS = 0;
        public static final int MAX_CLIMBER_HEIGHT_TICKS_VERTICAL = 0;
        public static final int MIN_CLIMBER_HEIGHT_TICKS = 0;
        public static final int WINCH_CIRCUMFERENCE_INCHES = 0;
        public static final int TICKS_PER_WINCH_ROTATION = 0;
    }

    public static class Intake {
        public static final int UPPER_MOTOR_ID = 0;
        public static final int LOWER_MOTOR_ID = 0;

        public static final double INTAKE_IN_SPEED_PCT = 0.5;
        public static final double INTAKE_OUT_SPEED_PCT = 0.5;
    }

    public static class Indexer {
        public static final int MOTOR_ID = 0;
        public static final double INDEX_SPEED_PCT = 0;
    }
  
    public static class Shamper {
        public static class Speed {
            public static final int UPPER_SHAMPER_SPEAKER_SPEED_PCT = 0;
            public static final int LOWER_SHAMPER_SPEAKER_SPEED_PCT = 0;

            public static final int UPPER_SHAMPER_SPEAKER_IDLE_SPEED_PCT = 0;
            public static final int LOWER_SHAMPER_SPEAKER_IDLE_SPEED_PCT = 0;

            public static final int UPPER_SHAMPER_AMP_SPEED_PCT = 0;
            public static final int LOWER_SHAMPER_AMP_SPEED_PCT = 0;
            
            public static final int UPPER_SHAMPER_AMP_IDLE_SPEED_PCT = 0;
            public static final int LOWER_SHAMPER_AMP_IDLE_SPEED_PCT = 0;
        }

        public static class Motors {
            public static final int LOWER_MOTOR_ID = 0;
            public static final int UPPER_MOTOR_ID = 0;
            public static final int PIVOT_MOTOR_ID = 0;
            public static final int PIVOT_ENCODER_ID = 0;
        }

    
        public static final int LOWER_SHOOTER_MOTOR_ID = 0;
        public static final int UPPER_SHOOTER_MOTOR_ID = 0;
        public static final int ROTATION_SHOOTER_MOTOR_ID = 0;
        public static final int ROTATION_ENCODER_ID = 0;
        public static final int INDEX_MOTOR_ID = 0;
        public static final int INDEXER_SENSOR_ID = 0;

        public static final int INDEX_SPEED_TPS = 0;
        public static final int LOWER_SHOOTER_IDLE_SPEED_TPS = 0;
        public static final int LOWER_SHOOTER_SPEAKER_SPEED_TPS = 0;
        public static final int LOWER_SHOOTER_AMP_SPEED_TPS = 0;
        public static final int UPPER_SHOOTER_IDLE_SPEED_TPS = 0;
        public static final int UPPER_SHOOTER_SPEAKER_SPEED_TPS = 0;
        public static final int UPPER_SHOOTER_AMP_SPEED_TPS = 0;

        public static final double UPPER_SHOOTER_KP = 0;
        public static final double UPPER_SHOOTER_KI = 0;
        public static final double UPPER_SHOOTER_KD = 0;

        public static final double LOWER_SHOOTER_KP = 0;
        public static final double LOWER_SHOOTER_KI = 0;
        public static final double LOWER_SHOOTER_KD = 0;

        public static final double ROTATION_SHOOTER_KP = 0;
        public static final double ROTATION_SHOOTER_KI = 0;
        public static final double ROTATION_SHOOTER_KD = 0;

        public static final double UPPER_SHOOTER_MAX_VELOCITY = 0;
        public static final double UPPER_SHOOTER_MAX_ACCELORATION = 0;
        
        public static final double LOWER_SHOOTER_MAX_VELOCITY = 0;
        public static final double LOWER_SHOOTER_MAX_ACCELORATION = 0;

        public static final double ROTATION_SHOOTER_MAX_VELOCITY = 0;
        public static final double ROTATION_SHOOTER_MAX_ACCELORATION = 0;

        public static final double ROTAION_MOTOR_TO_ACTUAL_ROTION_GEAR_RATIO = 0;

            public static final boolean UPPER_MOTOR_IS_INVERTED = false;
            public static final boolean LOWER_MOTOR_IS_INVERTED = false;
            public static final boolean ROTATION_MOTOR_IS_INVERTED = false;
            
            public static final double PIVOT_GEAR_RATIO = 0; 
            
            public static final double UPPER_MOTOR_KP = 0;
            public static final double UPPER_MOTOR_KI = 0;
            public static final double UPPER_MOTOR_KD = 0;

            public static final double LOWER_MOTOR_KP = 0;
            public static final double LOWER_MOTOR_KI = 0;
            public static final double LOWER_MOTOR_KD = 0;

            public static final double PIVOT_MOTOR_KP = 0;
            public static final double PIVOT_MOTOR_KI = 0;
            public static final double PIVOT_MOTOR_KD = 0;
            
            public static final double UPPER_MOTOR_MAX_VELOCITY = 0;
            public static final double LOWER_MOTOR_MAX_VELOCITY = 0;

            public static final double UPPER_MOTOR_MAX_ACCELERATION = 0;
            public static final double LOWER_MOTOR_MAX_ACCELERATION = 0;

            public static final double PIVOT_MOTOR_MAX_VELOCITY = 0;
            public static final double PIVOT_MOTOR_MAX_ACCELERATION = 0;
        
            public static final double TALONFX_TICS_PER_FULL_ROTATION = 0;

        public static class Angle {
            public static final double MINIMUM  = 0;
            public static final double MAXIMUM = 0;
            public static final double DEFAULT = 0;
            public static final double AMP = 0;
            public static final double CLIMB = 0;
        }

        public static final double SHOOTER_ANGLE_OFFSET_IN_DEGREES = 0;
    }

    public static class FieldAndRobot {
        public static final double SHAMPER_HEIGHT_IN_METERS = 0;
        public static final double SPEAKER_TARGET_HEIGHT_OFF_GROUND_IN_METERS = 0;

        public static final Translation2d RED_SPEAKER_LOCATION = new Translation2d(0, 0);
        public static final Translation2d BLUE_SPEAKER_LOCATION = new Translation2d(0, 0);

        public static final double GRAVITY_IN_METERS_PER_SECOND_SQUARED = 9.805665;

        public static final double NOTE_SPEED_IN_METERS_PER_SECOND = 0;

        public static final double SPEAKER_TARGET_VERTICAL_OFFSET_IN_METERS = 0;
        public static final double RED_SPEAKER_TARGET_X_OFFSET_IN_METERS = 0;
        public static final double RED_SPEAKER_TARGET_Y_OFFSET_IN_METERS = 0;
        public static final double BLUE_SPEAKER_TARGET_X_OFFSET_IN_METERS = 0;
        public static final double BLUE_SPEAKER_TARGET_Y_OFFSET_IN_METERS = 0;
    }

    public static class MusicPlayer {
        public static final TalonFX[] INSTRUMENT_TALONFX_PORT_LIST = {ShamperSubsystem.getUpperTalonFX(), ShamperSubsystem.getLowerTalonFX(), ShamperSubsystem.getRotationTalonFX()};
    }

    public static class Vision {
        public static final double NOTE_DETECTION_CAMERA_X_OFFSET = 0;
        public static final double NOTE_DETECTION_CAMERA_Y_OFFSET = 0;
        public static final double NOTE_DETECTION_CAMERA_ROTATION_OFFSET = 0;
    }

    public static class Drivetrain {
        // Left-to-right distance between drivetrain wheels
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0;
        // Front-to-back distance between drivetrain wheels
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0;

        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 0;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 0;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 0;
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET_RADIANS = 2.76420367321;

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 0;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 0;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 0;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET_RADIANS = 3.28720367321;

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 0;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 0;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 0;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET_RADIANS = 0.0412036732;

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 0;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 0;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 0;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET_RADIANS = -0.39179632679;

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
    public static final class PiCamera2 {
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
    }


    public static final class Elevator{
            public static final int BELT_MOTOR = 0;
    
            public static final double BELT_MOTOR_F = 0;
            public static final double BELT_MOTOR_P = 0;
            public static final double BELT_MOTOR_I = 0;
            public static final double BELT_MOTOR_D = 0;
    
            public static final double MANUAL_UP_SPEED = 0;
            public static final double MANUAL_DOWN_SPEED = 0;
            public static final double FEED_FORWARD = 0;
    
            public static final double BELT_MOTOR_CRUISE_VELOCITY = 0;
            public static final double BELT_MOTOR_MAX_ACCELERATION = 0;
            public static final int BELT_MOTOR_DEAD_ZONE_TICKS = 0;
        }

    public static final class TrapArm {
        public static final int ARM_MOTOR_ID = 0;
        public static final int ARM_ROTATION_MOTOR_ID = 0;

        public static final double ARM_KP = 0;
        public static final double ARM_KI = 0;
        public static final double ARM_KD = 0;
        public static final double ARM_MAX_VELOCITY = 0;
        public static final double ARM_MAX_ACCELERATION = 0;

        public static final double OUTTAKE_KP = 0;
        public static final double OUTTAKE_KI = 0;
        public static final double OUTTAKE_KD = 0;            
        public static final double OUTTAKE_MAX_VELOCITY = 0;
        public static final double OUTTAKE_MAX_ACCELERATION = 0;

        public static final double OUTTAKE_OUT_SPEED = 0;
    }   

    public static final class Dashboard {
        public static final String DRIVE_MODE_KEY = "Drive Mode";
        public static final String AUTO_COMPILED_KEY = "Auto Compiled";
    }
}


