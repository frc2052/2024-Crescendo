// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.team2052.lib.DrivetrainState;
import com.team2052.swervemodule.ModuleConfiguration;
import com.team2052.swervemodule.NeoSwerverModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;

public class DrivetrainSubsystem extends SubsystemBase {
    private RobotState robotState = RobotState.getInstance();

    private final NeoSwerverModule frontLeftModule;
    private final NeoSwerverModule frontRightModule;
    private final NeoSwerverModule backLeftModule;
    private final NeoSwerverModule backRightModule;

    private final AHRS navx;
    
    /** Creates a new SwerveDrivetrainSubsystem. */
    public DrivetrainSubsystem() {
        frontLeftModule = new NeoSwerverModule(
            "front left",
            ModuleConfiguration.MK4I_L3,
            Constants.Drivetrain.FRONT_LEFT_MODULE_DRIVE_MOTOR,
            Constants.Drivetrain.FRONT_LEFT_MODULE_STEER_MOTOR,
            Constants.Drivetrain.FRONT_LEFT_MODULE_STEER_ENCODER,
            new Rotation2d(Constants.Drivetrain.FRONT_LEFT_MODULE_STEER_OFFSET_RADIANS)
        );
        frontRightModule = new NeoSwerverModule(
            "front right",
            ModuleConfiguration.MK4I_L3,
            Constants.Drivetrain.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            Constants.Drivetrain.FRONT_RIGHT_MODULE_STEER_MOTOR,
            Constants.Drivetrain.FRONT_RIGHT_MODULE_STEER_ENCODER,
            new Rotation2d(Constants.Drivetrain.FRONT_RIGHT_MODULE_STEER_OFFSET_RADIANS)
        );
        backLeftModule = new NeoSwerverModule(
            "back left",
            ModuleConfiguration.MK4I_L3,
            Constants.Drivetrain.BACK_LEFT_MODULE_DRIVE_MOTOR,
            Constants.Drivetrain.BACK_LEFT_MODULE_STEER_MOTOR,
            Constants.Drivetrain.BACK_LEFT_MODULE_STEER_ENCODER,
            new Rotation2d(Constants.Drivetrain.BACK_LEFT_MODULE_STEER_OFFSET_RADIANS)
        );
        backRightModule = new NeoSwerverModule(
            "back right",
            ModuleConfiguration.MK4I_L3,
            Constants.Drivetrain.BACK_RIGHT_MODULE_DRIVE_MOTOR,
            Constants.Drivetrain.BACK_RIGHT_MODULE_STEER_MOTOR,
            Constants.Drivetrain.BACK_RIGHT_MODULE_STEER_ENCODER,
            new Rotation2d(Constants.Drivetrain.BACK_RIGHT_MODULE_STEER_OFFSET_RADIANS)
        );

        navx = new AHRS(SPI.Port.kMXP, (byte) 200);

        zeroGyro();
    }

    @Override
    public void periodic() {
        robotState.addDrivetrainState(new DrivetrainState(getModulePositions(), getNavx().getRotation2d()));
    }

    /**
     * All parameters are taken in normalized terms of [-1.0 to 1.0].
     */
    public void drive(
        double normalizedXVelocity, 
        double normalizedYVelocity, 
        double normalizedRotationVelocity, 
        boolean fieldCentric
    ) {
        normalizedXVelocity = Math.copySign(
            Math.min(Math.abs(normalizedXVelocity), 1.0),
            normalizedXVelocity
        );
        normalizedYVelocity = Math.copySign(
            Math.min(Math.abs(normalizedYVelocity), 1.0),
            normalizedYVelocity
        );
        normalizedRotationVelocity = Math.copySign(
            Math.min(Math.abs(normalizedRotationVelocity), 1.0),
            normalizedRotationVelocity
        );

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
            normalizedXVelocity * getMaxVelocityMetersPerSecond(), 
            normalizedYVelocity * getMaxVelocityMetersPerSecond(), 
            normalizedRotationVelocity * getMaxAngularVelocityRadiansPerSecond()
        );

        if (fieldCentric) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, robotState.getRotation2d());
        }

        drive(chassisSpeeds);
    }

    /**
     * Autonomous commands still require a drive method controlled via a ChassisSpeeds object
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = Constants.Drivetrain.kinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(swerveModuleStates);
    }

    public void stop() {
        drive(0, 0, 0, false);
    }

    public AHRS getNavx(){
        return navx;
    }

    public void zeroGyro() {
        navx.reset();
    }

    public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
        // Check if the wheels don't have a drive velocity to maintain the current wheel orientation.
        boolean hasVelocity = swerveModuleStates[0].speedMetersPerSecond != 0
            || swerveModuleStates[1].speedMetersPerSecond != 0 
            || swerveModuleStates[2].speedMetersPerSecond != 0
            || swerveModuleStates[3].speedMetersPerSecond != 0;

        frontLeftModule.setState(
            swerveModuleStates[0].speedMetersPerSecond, 
            hasVelocity ? swerveModuleStates[0].angle : frontLeftModule.getState().angle
        );
        frontRightModule.setState(
            swerveModuleStates[1].speedMetersPerSecond, 
            hasVelocity ? swerveModuleStates[1].angle : frontRightModule.getState().angle
        );
        backLeftModule.setState(
            swerveModuleStates[2].speedMetersPerSecond, 
            hasVelocity ? swerveModuleStates[2].angle : backLeftModule.getState().angle
        );
        backRightModule.setState(
            swerveModuleStates[3].speedMetersPerSecond, 
            hasVelocity ? swerveModuleStates[3].angle : backRightModule.getState().angle
        );
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition()
        };
    }

    public static double getMaxVelocityMetersPerSecond() {
        return NeoSwerverModule.getMaxVelocityMetersPerSecond(ModuleConfiguration.MK4I_L3);
    }

    public static double getMaxAngularVelocityRadiansPerSecond() {
        /*
         * Find the theoretical maximum angular velocity of the robot in radians per second 
         * (a measure of how fast the robot can rotate in place).
         */
        
        // return NeoSwerverModule.getMaxVelocityMetersPerSecond(ModuleConfiguration.MK4I_L3) / Math.hypot(
        //     Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, 
        //     Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0
        // );

        return 6 * Math.PI;
    }

    public void debug() {
        frontLeftModule.debug();
        frontRightModule.debug();   
        backLeftModule.debug();
        backRightModule.debug();
    }
}