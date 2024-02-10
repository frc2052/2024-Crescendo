// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.ShamperSubsystem;

public final class Constants {
    // TODO: Put in all the constants

    public static final class LED {
        public static final int CHANNEL_1_PIN = 0;
        public static final int CHANNEL_2_PIN = 0;
        public static final int CHANNEL_3_PIN = 0;
        public static final int CHANNEL_4_PIN = 0;
        public static final int CHANNEL_5_PIN = 0;
        public static final int CHANNEL_6_PIN = 0;
        public static final int CHANNEL_7_PIN = 0;
        public static final int CHANNEL_8_PIN = 0;
    }
    
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

        public static final double ROBOT_AIMING_ROTATION_OFFSET_IN_DEGREES = 0;
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


