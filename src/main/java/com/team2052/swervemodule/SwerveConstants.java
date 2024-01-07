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

    public static final class NeoSwerveModule {
        public static final int NEO_ROUNDS_PER_MINUTE = 5676;
    
        public static final double STEER_MOTOR_P = 1.0;
        public static final double STEER_MOTOR_I = 0.0;
        public static final double STEER_MOTOR_D = 0.1;
    }

    public static final class Falcon500SwerveModule {
        public static final int FALCON500_ROUNDS_PER_MINUTE = 6380;
        public static final int TICKS_PER_ROTATION = 2048;
        
        public static final double STEER_MOTOR_P = 0.2;
        public static final double STEER_MOTOR_I = 0.0;
        public static final double STEER_MOTOR_D = 0.1;
    }
}
