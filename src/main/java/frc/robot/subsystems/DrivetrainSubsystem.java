// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.team2052.swervemodule.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.RobotStateEstimator;
import frc.robot.util.states.DrivetrainState;

public class DrivetrainSubsystem extends SubsystemBase {
    private RobotState robotState = RobotState.getInstance();
    private ChassisSpeeds currentChassisSpeeds = new ChassisSpeeds();

    final SwerveModule frontLeftModule;
    final SwerveModule frontRightModule;
    final SwerveModule backLeftModule;
    final SwerveModule backRightModule;

    private final AHRS navx;
    
    /** Creates a new SwerveDrivetrainSubsystem. */
    public DrivetrainSubsystem() {
        frontLeftModule = new SwerveModule(
            "front left",
            Constants.CAN.FRONT_LEFT_MODULE_DRIVE_MOTOR,
            Constants.CAN.FRONT_LEFT_MODULE_STEER_MOTOR,
            Constants.CAN.FRONT_LEFT_MODULE_STEER_ENCODER,
            new Rotation2d(Constants.Drivetrain.FRONT_LEFT_MODULE_STEER_OFFSET_RADIANS)
        );
        backLeftModule = new SwerveModule(
            "back left",
            Constants.CAN.BACK_LEFT_MODULE_DRIVE_MOTOR,
            Constants.CAN.BACK_LEFT_MODULE_STEER_MOTOR,
            Constants.CAN.BACK_LEFT_MODULE_STEER_ENCODER,
            new Rotation2d(Constants.Drivetrain.BACK_LEFT_MODULE_STEER_OFFSET_RADIANS)
        );
        backRightModule = new SwerveModule(
            "back right",
            Constants.CAN.BACK_RIGHT_MODULE_DRIVE_MOTOR,
            Constants.CAN.BACK_RIGHT_MODULE_STEER_MOTOR,
            Constants.CAN.BACK_RIGHT_MODULE_STEER_ENCODER,
            new Rotation2d(Constants.Drivetrain.BACK_RIGHT_MODULE_STEER_OFFSET_RADIANS)
        );
        frontRightModule = new SwerveModule(
            "front right",
            Constants.CAN.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            Constants.CAN.FRONT_RIGHT_MODULE_STEER_MOTOR,
            Constants.CAN.FRONT_RIGHT_MODULE_STEER_ENCODER,
            new Rotation2d(Constants.Drivetrain.FRONT_RIGHT_MODULE_STEER_OFFSET_RADIANS)
        );

        navx = new AHRS(SPI.Port.kMXP, (byte) 200);
        navx.setAngleAdjustment(0);

        zeroOdometry();

        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
            () -> robotState.getRobotPoseAuto(), // Robot pose supplier for auot (correct range -180-180)
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            () -> robotState.getChassisSpeeds(), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            Constants.PathPlanner.HOLONOMIC_PATH_FOLLOWER_CONFIG,
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

    public void resetPose (Pose2d pose) {
        RobotStateEstimator.getInstance().resetOdometry(pose);
    }

    @Override
    public void periodic() {
        // currentChassisSpeeds = new ChassisSpeeds(navx.getVelocityX(), navx.getVelocityY)
        robotState.addDrivetrainState(new DrivetrainState(currentChassisSpeeds, getModulePositions(),  navx.getRotation2d()));
        debug();

        
        Logger.recordOutput("drivetrain omega radians", currentChassisSpeeds.omegaRadiansPerSecond);
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

        // The origin is always blue. When our alliance is red, X and Y need to be inverted
        var alliance = DriverStation.getAlliance();
        var invert = 1;
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            invert = -1;
        }

        if (fieldCentric) {
            // chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, navx.getRotation2d());
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                chassisSpeeds.vxMetersPerSecond * invert, 
                chassisSpeeds.vyMetersPerSecond * invert, 
                chassisSpeeds.omegaRadiansPerSecond, 
                RobotState.getInstance().getRobotPose().getRotation()
            );
        }

        drive(chassisSpeeds);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        // TODO: read navx values for chassis speeds instead
        currentChassisSpeeds = chassisSpeeds;
        SwerveModuleState[] swerveModuleStates = Constants.Drivetrain.kinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(swerveModuleStates);
    }

    public void stop() {
        drive(0, 0, 0, false);
    }

    public AHRS getNavx(){
        return navx;
    }

    public void zeroOdometry() {
        System.out.println("zeroing odometry");
        // if(RobotState.getInstance().isRedAlliance()){
        //     navx.setAngleAdjustment(180);
        // }

        // navx.reset();
        resetPose(new Pose2d(RobotState.getInstance().getRobotPose().getTranslation(), new Rotation2d()));
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
        backLeftModule.setState(
            swerveModuleStates[1].speedMetersPerSecond, 
            hasVelocity ? swerveModuleStates[1].angle : backLeftModule.getState().angle
        );
        backRightModule.setState(
            swerveModuleStates[2].speedMetersPerSecond, 
            hasVelocity ? swerveModuleStates[2].angle : backRightModule.getState().angle
        );
        frontRightModule.setState(
            swerveModuleStates[3].speedMetersPerSecond, 
            hasVelocity ? swerveModuleStates[3].angle : frontRightModule.getState().angle
        );
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition(),
            frontRightModule.getPosition()
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
        
        return SwerveModule.getMaxVelocityMetersPerSecond() / Math.hypot(
            Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, 
            Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0
        );

        //return 6 * Math.PI;
    }

    public void debug() {
        frontLeftModule.debug();
        backLeftModule.debug();
        backRightModule.debug();
        frontRightModule.debug();   
    }
}
