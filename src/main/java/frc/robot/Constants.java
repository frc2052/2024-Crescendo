// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.util.calculator.ShootAngleConfig;
import frc.robot.util.calculator.ShootingAngleCalculator;

public final class Constants {

    public static final double ROBOT_LENGTH_METERS = Units.inchesToMeters(29.25);
    public static final double ROBOT_WIDTH_METERS = Units.inchesToMeters(29.5);

    // TODO: Put in all the constants

    public static final class CAN {
        // Swerve Module CAN
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 10;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 11;
        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 12;

        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 1;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 2;
        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3;

        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 7;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 8;
        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 9;

        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 4;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 5;
        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 6;
        
        // Intake CAN
        public static final int INTAKE_MOTOR_ID = 13;

        // Indexer Spark Flex/Vortex CAN
        public static final int LOWER_INDEX_MOTOR_ID = 14;

        // Shooter CAN
        public static final int SHAMPER_INDEX_ID = 16;
        public static final int UPPER_SHOOTER_MOTOR_ID = 15;
        public static final int LOWER_SHOOTER_MOTOR_ID = 17;        

        // Climber CAN
        public static final int LEFT_CLIMBER_MOTOR = 19;
        public static final int RIGHT_CLIMBER_MOTOR = 21;
        
        // Shamper Pivot CAN
        public static final int LEFT_PIVOT_SHAMPER_MOTOR_ID = 18;
        public static final int RIGHT_PIVOT_SHAMPER_MOTOR_ID = 20;

    }

    public static class Trap {
        public static final int TRAP_RELAY_PIN = 0;
    }

    public static class Drivetrain {
        // Left-to-right distance between drivetrain wheels
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(24);
        // Front-to-back distance between drivetrain wheels
        public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(19);

        /*
         * FL: 157.8 FR:189.9 BL:3.5 BR:335.5
         */
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET_RADIANS = -Math.toRadians( 157.8); //2.76420367321;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET_RADIANS =  -Math.toRadians(3.5);//0.0412036732;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET_RADIANS = -Math.toRadians(335.5);//-0.39179632679;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET_RADIANS = -Math.toRadians(189.9);//3.28720367321;


        // public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        //     // Front left
        //     new Translation2d(Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
        //     // Front right
        //     new Translation2d(Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
        //     // Back left
        //     new Translation2d(-Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
        //     // Back right
        //     new Translation2d(-Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0)
        // );

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(0.302, 0.298),
            // Back left
            new Translation2d(-0.302, 0.178),
            // Back right
            new Translation2d(-0.302, -0.178),
            // Front right
            new Translation2d(0.302,  -0.298)
        );

        public static final double ROBOT_AIMING_ROTATION_OFFSET_IN_DEGREES = 0;
    }

    public static final class LED {
        public static final int CHANNEL_1_PIN = 0;
        public static final int CHANNEL_2_PIN = 0;
        public static final int CHANNEL_3_PIN = 0;
        public static final int CHANNEL_4_PIN = 0;
        public static final int CHANNEL_5_PIN = 0;
    }
    
    public static class MotorConstants {
        public static final double PIVOT_MOTOR_TICKS_PER_ROTATION = 42;
        public static final double FALCON500_TICKS_PER_ROTATION = 2048;
        public static final double FALCON500_MAX_RPM = 6380;
    }

    public static class Climber{
        public static final int CLIMBER_EXTENSION_HEIGHT_TICKS = 0;
        public static final int CLIMBER_RETRACTION_HEIGHT_TICKS = 0;
        public static final int MAX_CLIMBER_HEIGHT_TICKS_VERTICAL = 0;
        public static final int MIN_CLIMBER_HEIGHT_TICKS = 0;
        public static final int WINCH_CIRCUMFERENCE_INCHES = 0;
        public static final int TICKS_PER_WINCH_ROTATION = 0;

        public static final double CLIMBER_MOTOR_PCT = 1;
        public static final double CLIMBER_MOTOR_PCT_SLOW = 0.25;

        public static final boolean RIGHT_CLIMBER_MOTOR_INVERTED = false;
        public static final boolean LEFT_CLIMBER_MOTOR_INVERTED = false;
    }

    public static class Intake {
        public static final double INTAKE_IN_SPEED_PCT = -1;
        public static final double INTAKE_OUT_SPEED_PCT = 1;
    }

    public static class Indexer {
        public static final double LOWER_INDEX_SPEED_PCT = 1;
        public static final double UPPER_LOAD_SPEED_PCT = 0.7;
        public static final double UPPER_INDEX_SPEED_PCT = 1;
        
        public static final double AMP_INDEX_SPEED_PCT = 0.8;

        public static final double SLIDE_BACK_PCT = 0.2;

        public static final double UPPER_OUTTAKE_SPEED_PCT = 1;

        // DIO
        public static final int INDEXER_SENSOR_PIN = 0;
    }
  
    public static class Shamper {

        // DIO pins
        public static final int ROTATION_ENCODER_PIN = 1;
        public static final int LIMIT_SWITCH_PIN = 2;
        public static final int AMP_HALL_EFFECT_PIN = 0;
        public static final int PODIUM_HALL_EFFECT_PIN = 0;

        public static final double ROTATION_SHOOTER_KP = 0;
        public static final double ROTATION_SHOOTER_KI = 0;
        public static final double ROTATION_SHOOTER_KD = 0;

        public static final boolean UPPER_MOTOR_IS_INVERTED = true;
        public static final boolean LOWER_MOTOR_IS_INVERTED = false;
        public static final boolean LEFT_PIVOT_MOTOR_IS_INVERTED = false;
        public static final boolean RIGHT_PIVOT_MOTOR_IS_INVERTED = false;
            
        public static final double PIVOT_GEAR_RATIO = 60 * (48 / 22); 
            
        //edit these later
        public static final double UPPER_MOTOR_KP = 0.2;
        public static final double UPPER_MOTOR_KI = 0.002;
        public static final double UPPER_MOTOR_KD = 1;

        public static final double LOWER_MOTOR_KP = 0.2;
        public static final double LOWER_MOTOR_KI = 0.002;
        public static final double LOWER_MOTOR_KD = 1;

        public static final double PIVOT_MOTOR_KP = 0.2;
        public static final double PIVOT_MOTOR_KI = 0.002;
        public static final double PIVOT_MOTOR_KD = 1;

        public static final double SHOOTER_MAX_VELOCITY_RPS = 100;
        
        public static final double UPPER_SHAMPER_SPEAKER_SPEED_RPS = 0.9 * SHOOTER_MAX_VELOCITY_RPS;
        public static final double LOWER_SHAMPER_SPEAKER_SPEED_RPS = 0.9 * SHOOTER_MAX_VELOCITY_RPS;

        public static final double UPPER_SHAMPER_SPEAKER_IDLE_SPEED_RPS = 0.7 * SHOOTER_MAX_VELOCITY_RPS;
        public static final double LOWER_SHAMPER_SPEAKER_IDLE_SPEED_RPS = 0.7 * SHOOTER_MAX_VELOCITY_RPS;

        public static final double UPPER_SHAMPER_SUB_SPEED_RPS = 0.7 * SHOOTER_MAX_VELOCITY_RPS;
        public static final double LOWER_SHAMPER_SUB_SPEED_RPS = 0.7 * SHOOTER_MAX_VELOCITY_RPS;

        public static final double UPPER_SHAMPER_AMP_SPEED_PCT = -1 * SHOOTER_MAX_VELOCITY_RPS;
        public static final double LOWER_SHAMPER_AMP_SPEED_PCT = 0.20 * SHOOTER_MAX_VELOCITY_RPS;
        
        public static final double UPPER_SHAMPER_AMP_IDLE_SPEED_RPS = 0;
        public static final double LOWER_SHAMPER_AMP_IDLE_SPEED_RPS = 0;

        public static final double UPPER_SHAMPER_TRAP_SPEED_RPS = 0.12 * SHOOTER_MAX_VELOCITY_RPS;
        public static final double LOWER_SHAMPER_TRAP_SPEED_RPS = 0.12 * SHOOTER_MAX_VELOCITY_RPS;
        
        public static final double INDEX_SPEED_TPS = 0;

        public static final double PIVOT_MOTOR_MANUAL_VELOCITY = 0.25;
        public static final double PIVOT_MOTOR_MAX_VELOCITY = 0.5;
        public static final double PIVOT_MOTOR_LEVEL_2_VELOCITY = 0.1;
        public static final double PIVOT_MOTOR_LEVEL_1_VELOCITY = 0.1;
        public static final double PIVOT_MOTOR_MAX_ACCELERATION = 0;

        public static class Angle {
            public static final double MINIMUM  = 12;
            public static final double MAXIMUM = 124;
            public static final double CLIMB = 13;
            public static final double DEFAULT = 30;
            public static final double PODIUM = 34;
            public static final double AMP = 122;
            public static final double SUB = 53;
            public static final double TRAP = 95;
        }

        public static final double ENCODER_OFFSET_DEGREES = 0;
        public static final double DEAD_ZONE_DEGREES = 1;
        public static final double DEAD_ZONE_SHOOTER_SPEED_RPS = 2;
        public static final double SHOOTER_TOLERANCE_PERCENT = 0.01;
    }

    public static class FieldAndRobot {
        public static final double SHAMPER_HEIGHT_IN_METERS = Units.inchesToMeters(17);
        public static final double SPEAKER_TARGET_HEIGHT_OFF_GROUND_IN_METERS = 1.98;

        //got from cad model
        public static final Translation2d RED_SPEAKER_LOCATION = new Translation2d(Units.inchesToMeters(651.157), Units.inchesToMeters(218.416));
        public static final Translation2d BLUE_SPEAKER_LOCATION = new Translation2d(Units.inchesToMeters(0.066), Units.inchesToMeters(218.415));

        public static final double GRAVITY_IN_METERS_PER_SECOND_SQUARED = 9.805665;

        public static final double NOTE_SPEED_IN_METERS_PER_SECOND = 10; //0 will cause an error

        public static final double SPEAKER_TARGET_VERTICAL_OFFSET_IN_METERS = 0;
        public static final double RED_SPEAKER_TARGET_X_OFFSET_IN_METERS = -0.3;
        public static final double RED_SPEAKER_TARGET_Y_OFFSET_IN_METERS = 0;
        public static final double BLUE_SPEAKER_TARGET_X_OFFSET_IN_METERS = 0.3;
        public static final double BLUE_SPEAKER_TARGET_Y_OFFSET_IN_METERS = 0;
    }

    public static class MusicPlayer {
        public static final TalonFX[] INSTRUMENT_TALONFX_PORT_LIST = {ShamperSubsystem.getUpperTalonFX(), ShamperSubsystem.getLowerTalonFX()};
    }

    public static class Vision {
        public static final double NOTE_DETECTION_CAMERA_X_OFFSET = 0;
        public static final double NOTE_DETECTION_CAMERA_Y_OFFSET = 0;
        public static final double NOTE_DETECTION_CAMERA_ROTATION_OFFSET = 0;
    }
    public static final class PhotonCamera1 {
        public static final String CAMERA_NAME = "Arducam_OV9281_USB_Cam_001";

        public static final double X_OFFSET_M = 0.29;
        public static final double Y_OFFSET_M = -0.26;
        public static final double Z_OFFSET_M = 0.25;

        public static final double THETA_X_OFFSET_DEGREES = 0.0; // roll
        public static final double THETA_Y_OFFSET_DEGREES = -29.536; // pitch
        public static final double THETA_Z_OFFSET_DEGREES = 0.0; // yaw

        public static final Transform3d ROBOT_TO_CAMERA_METERS = new Transform3d(
            new Translation3d(X_OFFSET_M, Y_OFFSET_M, Z_OFFSET_M), 
            new Rotation3d(Units.degreesToRadians(THETA_X_OFFSET_DEGREES), Units.degreesToRadians(THETA_Y_OFFSET_DEGREES), Units.degreesToRadians(THETA_Z_OFFSET_DEGREES))
        );
    }
    public static final class PhotonCamera2 {
        // TODO: make offsets more precise than caleb's eyeballing
        public static final String CAMERA_NAME = "Arducam_OV9281_USB_Cam_002";

        public static final double X_OFFSET_M = 0.01;
        public static final double Y_OFFSET_M = -0.40;
        public static final double Z_OFFSET_M = 0.31;

        public static final double THETA_X_OFFSET_DEGREES = 0.0; // roll
        public static final double THETA_Y_OFFSET_DEGREES = -15; // pitch
        public static final double THETA_Z_OFFSET_DEGREES = 0.0; // yaw

        public static final Transform3d ROBOT_TO_CAMERA_METERS = new Transform3d(
            new Translation3d(X_OFFSET_M, Y_OFFSET_M, Z_OFFSET_M), 
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
    
            public static final double BELT_MOTOR_F = 0.5;
            public static final double BELT_MOTOR_P = 0.2;
            public static final double BELT_MOTOR_I = 0.002;
            public static final double BELT_MOTOR_D = 1;
    
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

        public static final double ARM_KP = 0.2;
        public static final double ARM_KI = 0.002;
        public static final double ARM_KD = 1;
        public static final double ARM_MAX_VELOCITY = 0;
        public static final double ARM_MAX_ACCELERATION = 0;

        public static final double OUTTAKE_KP = 0.2;
        public static final double OUTTAKE_KI = 0.002;
        public static final double OUTTAKE_KD = 1;            
        public static final double OUTTAKE_MAX_VELOCITY = 0;
        public static final double OUTTAKE_MAX_ACCELERATION = 0;

        public static final double OUTTAKE_OUT_SPEED = 0;
    }   

    public static final class Dashboard {
        public static final String DRIVE_MODE_KEY = "Drive Mode";
        public static final String AUTO_COMPILED_KEY = "Auto Compiled";
    }

    public static final class PathPlanner {
        public static final double TRANSLATION_KP = .7;
        public static final double TRANSLATION_KI = 0;
        public static final double TRANSLATION_KD = 0;

        public static final double ROTATION_KP = .7;
        public static final double ROTATION_KI = 0;
        public static final double ROTATION_KD = 0;

        public static final double MAX_MODULE_SPEED = 4.5;
        public static final double DRIVE_BASE_RADIUS_METERS = Math.hypot(Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS, Drivetrain.DRIVETRAIN_WHEELBASE_METERS) / 2;

        public static final HolonomicPathFollowerConfig HOLONOMIC_PATH_FOLLOWER_CONFIG = 
            new HolonomicPathFollowerConfig(
                    new PIDConstants( 
                        TRANSLATION_KP, 
                        TRANSLATION_KI, 
                        TRANSLATION_KD
                    ), // Translation PID constants
                    new PIDConstants(
                        ROTATION_KP, 
                        ROTATION_KI, 
                        ROTATION_KD
                    ), // Rotation PID constants
                    MAX_MODULE_SPEED, // Max module speed, in m/s
                    DRIVE_BASE_RADIUS_METERS, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            );
    }

    public static final class LEDs {
        // Binary arduino code output bits
        public static final int CHANNEL_1_PIN = 3; // 2^0
        public static final int CHANNEL_2_PIN = 4; // 2^1
        public static final int CHANNEL_3_PIN = 5; // 2^2
        public static final int CHANNEL_4_PIN = 6; // 2^3
        public static final int CHANNEL_5_PIN = 7; // 2^4
        public static final int CHANNEL_6_PIN = 8; // 2^5
        public static final int CHANNEL_7_PIN = 9; // 2^6
    }
}


