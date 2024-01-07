package com.team2052.lib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class DrivetrainState {
    SwerveModulePosition[] swerveModulePositions;
    Rotation2d rotation2d;

    public DrivetrainState(
        SwerveModulePosition[] swerveModulePositions,
        Rotation2d rotation2d
    ) {
        this.swerveModulePositions = swerveModulePositions;
        this.rotation2d = rotation2d;
    }

    public SwerveModulePosition[] getModulePositions(){
        return swerveModulePositions;
    }

    public Rotation2d getRotation2d(){
        return rotation2d;
    }
}
