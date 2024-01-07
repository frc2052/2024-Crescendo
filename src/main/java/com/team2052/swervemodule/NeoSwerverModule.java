// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2052.swervemodule;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Swerve module implementation for swerve module with Neos
 */
public class NeoSwerverModule extends SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax steerMotor;

    public NeoSwerverModule(
        String debugName, 
        ModuleConfiguration moduleConfiguration,
        int driveMotorChannel,
        int steerMotorChannel,
        int canCoderChannel,
        Rotation2d steerOffset
    ) {
        super(debugName, moduleConfiguration, canCoderChannel, steerOffset);

        /*
         * Drive Motor Initialization
         */
        driveMotor = new CANSparkMax(driveMotorChannel, CANSparkMaxLowLevel.MotorType.kBrushless);
        checkError("Failed to resotre drive motor factory defaults", driveMotor.restoreFactoryDefaults());

        // Reduce CAN status frame rates
        checkError(
            "Failed to set drive motor periodic status frame rate",
            driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100),
            driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20),
            driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20)
        );

        checkError(
            "Failed to set drive motor idle mode",
            driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake)
        );
        driveMotor.setInverted(moduleConfiguration.isDriveInverted());

        checkError(
            "Failed to enable drive motor voltage compensation",
            driveMotor.enableVoltageCompensation(SwerveConstants.MAX_VOLTAGE_VOLTS)
        );
        checkError(
            "Failed to set steer motor current limit",
            driveMotor.setSmartCurrentLimit(
                SwerveConstants.DRIVE_STALL_CURRENT_LIMIT_AMPS, 
                SwerveConstants.DRIVE_FREE_CURRENT_LIMIT_AMPS
            )
        );

        // Drive Motor encoder initialization
        RelativeEncoder driveEncoder = driveMotor.getEncoder();

        // Conversion factor for switching between ticks and meters in terms of meters per tick
        double drivePositionConversionFactor = Math.PI * moduleConfiguration.getWheelDiameter() * 
            moduleConfiguration.getDriveReduction();
        
        checkError(
            "Failed to set drive motor encoder conversion factors",
            // Set the position conversion factor so the encoder will automatically convert ticks to meters
            driveEncoder.setPositionConversionFactor(drivePositionConversionFactor),
            // Velocity of the encoder in meters per second
            driveEncoder.setVelocityConversionFactor(drivePositionConversionFactor / 60.0)
        );

        /*
         * Steer Motor Initialization
         */
        steerMotor = new CANSparkMax(steerMotorChannel, CANSparkMaxLowLevel.MotorType.kBrushless);
        checkError("Failed to resotre steer motor factory defaults", steerMotor.restoreFactoryDefaults());

        // Reduce CAN status frame rates
        checkError(
            "Failed to set steer motor periodic status frame rate",
            steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100),
            steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20),
            steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20)
        );

        checkError(
            "Failed to set steer motor idle mode",
            steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake)
        );
        steerMotor.setInverted(!moduleConfiguration.isSteerInverted());

        checkError(
            "Failed to enable steer motor voltage compensation",
            steerMotor.enableVoltageCompensation(SwerveConstants.MAX_VOLTAGE_VOLTS)
        );

        checkError(
            "Failed to set steer motor current limit",
            steerMotor.setSmartCurrentLimit((int) SwerveConstants.STEER_CURRENT_LIMIT_AMPS)
        );

        
        // Steer Motor encoder initialization
        RelativeEncoder steerEncoder = steerMotor.getEncoder();

        // Conversion factor for switching between ticks and radians in terms of radians per tick
        double steerPositionConversionFactor = 2.0 * Math.PI * moduleConfiguration.getSteerReduction();

        checkError(
            "Failed to set drive motor encoder conversion factors",
            // Set the position conversion factor so the encoder will automatically convert ticks to radians
            steerEncoder.setPositionConversionFactor(steerPositionConversionFactor),
            // Velocity of the encoder in radians per second
            steerEncoder.setVelocityConversionFactor(steerPositionConversionFactor / 60.0)
        );

        // Sets the steer motor encoder to the absolute position of the CANCoder for startup orientation
        checkError(
            "Failed to set steer motor encoder position",
            steerEncoder.setPosition(Math.toRadians(canCoder.getAbsolutePosition()))
        );

        SparkMaxPIDController steerController = steerMotor.getPIDController();
        checkError(
            "Failed to configure steer motor PID",
            steerController.setP(SwerveConstants.NeoSwerveModule.STEER_MOTOR_P),
            steerController.setI(SwerveConstants.NeoSwerveModule.STEER_MOTOR_I),
            steerController.setD(SwerveConstants.NeoSwerveModule.STEER_MOTOR_D),
            steerController.setPositionPIDWrappingMinInput(-Math.PI),
            steerController.setPositionPIDWrappingMaxInput(Math.PI),
            steerController.setPositionPIDWrappingEnabled(true)
        );

        checkError(
            "Failed to set steer motor PID feedback device",
            steerController.setFeedbackDevice(steerEncoder)
        );
    }

    @Override
    public SwerveModuleState getState() {
        // Both encoder values are automatically in units of meters per second and
        // radians because of the position and velocity conversion factors
        return new SwerveModuleState(
            driveMotor.getEncoder().getVelocity(),
            new Rotation2d(
                steerMotor.getEncoder().getPosition() % (2.0 * Math.PI)
            )
        );
    }

    @Override
    public void setState(double velocityMetersPerSecond, Rotation2d steerAngle) {
        SwerveModuleState desiredState = new SwerveModuleState(velocityMetersPerSecond, steerAngle);
        // Reduce radians to 0 to 2pi range and simplify to nearest angle
        desiredState = SwerveModuleState.optimize(
            desiredState,
            getState().angle
        );

        // Set the motor to our desired velocity as a percentage of our max velocity
        driveMotor.set(
            desiredState.speedMetersPerSecond / getMaxVelocityMetersPerSecond(moduleConfiguration)
        );

        SmartDashboard.putNumber(debugName + ": Speed", desiredState.speedMetersPerSecond);

        steerMotor.getPIDController().setReference(
            desiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition
        );
        
        SmartDashboard.putNumber(debugName + ": Desired Rotation", steerAngle.getDegrees());
        SmartDashboard.putNumber(debugName + ": Rotation", Math.toDegrees(steerMotor.getEncoder().getPosition()));
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveMotor.getEncoder().getPosition(),
            new Rotation2d(
                steerMotor.getEncoder().getPosition()
            )
        );
    }

    public static double getMaxVelocityMetersPerSecond(ModuleConfiguration moduleConfiguration) {
        /*
         * The formula for calculating the theoretical maximum velocity is:
         * [Motor free speed (RPM)] / 60 * [Drive reduction] * [Wheel diameter (m)] * pi
         * This is a measure of how fast the robot should be able to drive in a straight line.
         */
        return SwerveConstants.NeoSwerveModule.NEO_ROUNDS_PER_MINUTE / 60 * moduleConfiguration.getDriveReduction() * 
            moduleConfiguration.getWheelDiameter() * Math.PI;
    }
}
