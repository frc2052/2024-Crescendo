// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OverTheBumperIntake{
    public static final int INSOLENOID_ID = 0;
    public static final int OUTSOLENOID_ID = 0;
    public static final int INTAKE_MOTOR_ID = 0;
  }
    public static class Climber{
      public static final int COMPRESSOR_MODULE_ID = 0;
      public static final int CLIMBER_FORWARD_SOLENOID = 0;
      public static final int CLIMBER_BACKWARD_SOLENOID = 0;
      public static final int CLIMBER_LOCK_SOLENOID = 0;
      public static final int CLIMBER_UNLOCK_SOLENOID = 0;
      public static final int CLIMBER_EXTENSION_SPEED_PCT = 0;
      public static final int CLIMBER_MOTOR = 0;
      public static final int MAX_CLIMBER_HEIGHT_TICKS_VERTICAL = 0;
      public static final int MIN_CLIMBER_HEIGHT_TICKS = 0;
      public static final int WINCH_CIRCUMFERENCE_INCHES = 0;
      public static final int TICKS_PER_WINCH_ROTATION = 0;
      public static final int MAX_CLIMBER_HEIGHT_TICKS_TILTED = 0;

  public static class UnderIntake1SideConstants {
    public final static int UPPER_MOTOR_CHANNEL = 1;
    public final static int LOWER_MOTOR_CHANNEL = 2;

    public final static double INTAKE_IN_SPEED_TPS = 0.5;
    public final static double INTAKE_OUT_SPEED_TPS = 0.5;
  }
  
    public class HorizontalShooterConstants{
        public static final int LEFT_SHOOTER_MOTOR_ID = 0; //change later
        public static final int RIGHT_SHOOTER_MOTOR_ID = 0; //change later
        public static final double SHOOTER_DEFAULT_SPEED_TPS = 0.5; //change later
    }

    public static class VerticalShooterConstants {
        public static final int LOWER_SHOOTER_MOTOR_ID = 0; //CHANGE
        public static final int UPPER_SHOOTER_MOTOR_ID = 0; //CHANGE
        public static final double SHOOTER_IDLE_SPEED_TPS = 10000; //CHANGE
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
}

