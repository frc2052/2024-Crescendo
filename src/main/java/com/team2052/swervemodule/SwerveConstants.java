// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2052.swervemodule;

/** 
 * Constants for swerve modules.
 */
public final class SwerveConstants {
    public static final double MAX_VOLTAGE_VOLTS = 12.0;
    public static final int DRIVE_STALL_CURRENT_LIMIT_AMPS = 40;
    public static final int DRIVE_FREE_CURRENT_LIMIT_AMPS = 60;
    public static final double STEER_CURRENT_LIMIT_AMPS = 20.0;
    public static final int CAN_TIMEOUT_MS = 250;

    public static final class SwerveModule {
        public static final int NEO_ROUNDS_PER_MINUTE = 5676;
    
        public static final double STEER_MOTOR_P = 1.0;
        public static final double STEER_MOTOR_I = 0.0;
        public static final double STEER_MOTOR_D = 0.1;

        // L3
        // public static final double WHEEL_DIAMETER_METERS = 0.10033;
        // public static final double DRIVE_REDUCTION = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);
        // public static final boolean DRIVE_INVERTED = true;
        // public static final double STEER_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);
        // public static final boolean STEER_INVERTED = false;

        // L2
        public static final double WHEEL_DIAMETER_METERS = 0.10033;
        public static final double DRIVE_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
        public static final boolean DRIVE_INVERTED = true;
        public static final double STEER_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);
        public static final boolean STEER_INVERTED = false;
    }
}
