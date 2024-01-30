// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
    // TODO: Put in all the constants

    public static class OverTheBumperIntake{
        public static final int INSOLENOID_ID = 0;
        public static final int OUTSOLENOID_ID = 0;
        public static final int INTAKE_MOTOR_ID = 0;
    }
    public static class Climber{
        public static final int CLIMBER_EXTENSION_HEIGHT_TICKS = 0;
        public static final int CLIMBER_RETRACTION_HEIGHT_TICKS = 0;
        public static final int CLIMBER_MOTOR = 0;
        public static final int MAX_CLIMBER_HEIGHT_TICKS_VERTICAL = 0;
        public static final int MIN_CLIMBER_HEIGHT_TICKS = 0;
        public static final int WINCH_CIRCUMFERENCE_INCHES = 0;
        public static final int TICKS_PER_WINCH_ROTATION = 0;
    }

    public static class OneUnderBumperIntake {
        public final static int UPPER_MOTOR_CHANNEL = 1;
        public final static int LOWER_MOTOR_CHANNEL = 2;

        public final static double INTAKE_IN_SPEED_TPS = 0.5;
        public final static double INTAKE_OUT_SPEED_TPS = 0.5;
    }
  
    public static class VerticalShooter {
        public static final int LOWER_SHOOTER_MOTOR_ID = 0;
        public static final int UPPER_SHOOTER_MOTOR_ID = 0;
        public static final int ROTATION_SHOOTER_MOTOR_ID = 0;
        public static final int ROTATION_ENCODER_ID = 0;

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

        public static final boolean UPPER_MOTOR_IS_INVERTED = false;
        public static final boolean LOWER_MOTOR_IS_INVERTED = false;
        public static final boolean ROTATION_MOTOR_IS_INVERTED = false;

        public static final double SHOOTER_MINNIMUM_ANGLE = 0;
        public static final double SHOOTER_MAXIMUM_ANGLE = 0;
        public static final double SHOOTER_DEFAULT_ANGLE = 0;

        public static final double TICKS_PER_TALONFX_FULL_ROTION = 0;
        public static final double ROTAION_MOTOR_TO_ACTUAL_ROTION_GEAR_RATIO = 0; //change

        public static final double SHOOTER_AMP_SCORING_ANGLE = 0;
    }

    public static class FeildAndRobot {
        public static final double SHOOTER_HIGHT_IN_METERS = 0;
        public static final double SPEAKER_TARGET_HIGHT_OFF_GROUND_IN_METERS = 0;

        public static final Translation2d RED_SPEAKER_LOCATION = new Translation2d(0, 0);
        public static final Translation2d BLUE_SPEAKER_LOCATION = new Translation2d(0, 0);
    }

    public static class MusicPlayer {
        public static final int[] INSTRAMENT_TALONFX_PORT_LIST = {0, 0, 0};
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

     public static final class Elevator{
            public static final int BELT_MOTOR = 15;
    
            public static final int LIMIT_SWITCH_DIO_CHANNEL = 0;
    
            public static final double BELT_MOTOR_F = 0.054;
            public static final double BELT_MOTOR_P = 0.2046;
            public static final double BELT_MOTOR_I = 0.002;
            public static final double BELT_MOTOR_D = 1.023;
    
            public static final double MANUAL_UP_SPEED = 0.15;
            public static final double MANUAL_DOWN_SPEED = -0.15;
            public static final double FEED_FORWARD = 0.065;
    
            public static final double BELT_MOTOR_CRUISE_VELOCITY = 14000;
            public static final double BELT_MOTOR_MAX_ACCELERATION = 24000;
            public static final int BELT_MOTOR_DEAD_ZONE_TICKS = 250;
        }

        public static final class Dashboard {
            public static final String DRIVE_MODE_KEY = "Drive Mode";
            public static final String ELEVATOR_POSITION_KEY = "Elevator Position";
            public static final String ELEVATOR_LIMIT_SWITCH_KEY = "Elevator Limit Switch"; 
            public static final String INTAKE_CURRENT_KEY = "Intake Current";
            public static final String PRESSURE_KEY = "Pressure";
            public static final String CAMERA_CONNECTION_KEY = "Camera Connected";
            public static final String AUTO_COMPILED_KEY = "Auto Compiled";
            public static final String AUTO_DESCRIPTION_KEY = "Auto Description";
        }
}

