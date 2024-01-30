// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.team2052.lib.DrivetrainState;
import com.team2052.swervemodule.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;

public class DrivetrainSubsystem extends SubsystemBase {
    private RobotState robotState = RobotState.getInstance();

    final SwerveModule frontLeftModule;
    final SwerveModule frontRightModule;
    final SwerveModule backLeftModule;
    final SwerveModule backRightModule;

    private final AHRS navx;
    
    /** Creates a new SwerveDrivetrainSubsystem. */
    public DrivetrainSubsystem() {
        frontLeftModule = new SwerveModule(
            "front left",
            Constants.Drivetrain.FRONT_LEFT_MODULE_DRIVE_MOTOR,
            Constants.Drivetrain.FRONT_LEFT_MODULE_STEER_MOTOR,
            Constants.Drivetrain.FRONT_LEFT_MODULE_STEER_ENCODER,
            new Rotation2d(Constants.Drivetrain.FRONT_LEFT_MODULE_STEER_OFFSET_RADIANS)
        );
        frontRightModule = new SwerveModule(
            "front right",
            Constants.Drivetrain.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            Constants.Drivetrain.FRONT_RIGHT_MODULE_STEER_MOTOR,
            Constants.Drivetrain.FRONT_RIGHT_MODULE_STEER_ENCODER,
            new Rotation2d(Constants.Drivetrain.FRONT_RIGHT_MODULE_STEER_OFFSET_RADIANS)
        );
        backLeftModule = new SwerveModule(
            "back left",
            Constants.Drivetrain.BACK_LEFT_MODULE_DRIVE_MOTOR,
            Constants.Drivetrain.BACK_LEFT_MODULE_STEER_MOTOR,
            Constants.Drivetrain.BACK_LEFT_MODULE_STEER_ENCODER,
            new Rotation2d(Constants.Drivetrain.BACK_LEFT_MODULE_STEER_OFFSET_RADIANS)
        );
        backRightModule = new SwerveModule(
            "back right",
            Constants.Drivetrain.BACK_RIGHT_MODULE_DRIVE_MOTOR,
            Constants.Drivetrain.BACK_RIGHT_MODULE_STEER_MOTOR,
            Constants.Drivetrain.BACK_RIGHT_MODULE_STEER_ENCODER,
            new Rotation2d(Constants.Drivetrain.BACK_RIGHT_MODULE_STEER_OFFSET_RADIANS)
        );

        navx = new AHRS(SPI.Port.kMXP, (byte) 200);

        zeroGyro();

        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
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
        return SwerveModule.getMaxVelocityMetersPerSecond();
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

    //TODO: Make real getPose, resetPose, getSpeeds, and driveRobotRelative methods
    public Pose2d getPose() {return new Pose2d();}
    public void resetPose(Pose2d pose) {}
    public ChassisSpeeds getRobotRelativeSpeeds() {return new ChassisSpeeds();}
    public void driveRobotRelative(ChassisSpeeds speeds) {}
}
