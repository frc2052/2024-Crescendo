package frc.robot.util.states;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class DrivetrainState {
    ChassisSpeeds chassisSpeeds;
    SwerveModulePosition[] swerveModulePositions;
    Rotation2d rotation2d;

    public DrivetrainState(
        ChassisSpeeds chassisSpeeds,
        SwerveModulePosition[] swerveModulePositions,
        Rotation2d rotation2d
    ) {
        this.chassisSpeeds = chassisSpeeds;
        this.swerveModulePositions = swerveModulePositions;
        this.rotation2d = rotation2d;
    }

    public SwerveModulePosition[] getModulePositions(){
        return swerveModulePositions;
        }

    public Rotation2d getRotation2d(){
        return rotation2d;
    }

    public ChassisSpeeds getChassisSpeeds(){
        return chassisSpeeds;
    }
}
