// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2052.swervemodule;

/** 
 * All different possible configurations of Swerve Drive Specialties modules.
 */
public enum ModuleConfiguration {
    MK3_STANDARD(
        0.1016,
        (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 60.0),
        true,
        (15.0 / 32.0) * (10.0 / 60.0),
        true
    ),
    MK3_FAST(
        0.1016,
        (16.0 / 48.0) * (28.0 / 16.0) * (15.0 / 60.0),
        true,
        (15.0 / 32.0) * (10.0 / 60.0),
        true
    ),
    MK4_L1(
        0.10033,
        (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0),
        true,
        (15.0 / 32.0) * (10.0 / 60.0),
        true
    ),
    MK4_L2(
        0.10033,
        (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
        true,
        (15.0 / 32.0) * (10.0 / 60.0),
        true
    ),
    MK4_L3(
        0.10033,
        (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0),
        true,
        (15.0 / 32.0) * (10.0 / 60.0),
        true
    ),
    MK4_L4(
        0.10033,
        (16.0 / 48.0) * (28.0 / 16.0) * (15.0 / 45.0),
        true,
        (15.0 / 32.0) * (10.0 / 60.0),
        true
    ),
    MK4I_L1(
        0.10033,
        (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0),
        true,
        (14.0 / 50.0) * (10.0 / 60.0),
        false
    ),
    MK4I_L2(
        0.10033,
        (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
        true,
        (14.0 / 50.0) * (10.0 / 60.0),
        false
    ),
    MK4I_L3(
        0.10033,
        (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0),
        true,
        (14.0 / 50.0) * (10.0 / 60.0),
        false
    );
    
    private final double wheelDiameter;
    private final double driveReduction;
    private final boolean driveInverted;
    private final double steerReduction;
    private final boolean steerInverted;

    private ModuleConfiguration(
        double wheelDiameter, 
        double driveReduction, 
        boolean driveInverted,
        double steerReduction, 
        boolean steerInverted
    ) {
        this.wheelDiameter = wheelDiameter;
        this.driveReduction = driveReduction;
        this.driveInverted = driveInverted;
        this.steerReduction = steerReduction;
        this.steerInverted = steerInverted;
    }

    public double getWheelDiameter() {
        return wheelDiameter;
    }

    public double getDriveReduction() {
        return driveReduction;
    }

    public boolean isDriveInverted() {
        return driveInverted;
    }

    public double getSteerReduction() {
        return steerReduction;
    }

    public boolean isSteerInverted() {
        return steerInverted;
    }
}
