package com.team2052.swervemodule;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class KrakenSwerveModule extends SwerveModule {
    private final TalonFX driveMotor;
    private final CANSparkMax steerMotor;
    private double driveVelocityConversionFactor;
    private double drivePositionConversionFactor;

    public KrakenSwerveModule(
            String debugName, 
            ModuleConfiguration moduleConfiguration, 
            int driveMotorChannel,
            int steerMotorChannel,
            int canCoderChannel,
            Rotation2d steerOffset
        ) {
        super(debugName, moduleConfiguration, canCoderChannel, steerOffset);

        /*
         *  Drive Motor Initialization
         */

        driveMotor = new TalonFX(driveMotorChannel);

        checkError("Failed to restore drive motor factory defaults", driveMotor.getConfigurator().apply(new TalonFXConfiguration()));
        driveMotor.setInverted(SwerveConstants.KrakenSwerveModule.DRIVE_INVERTED);
        driveMotor.setNeutralMode(NeutralModeValue.Brake);


        drivePositionConversionFactor = Math.PI * moduleConfiguration.getWheelDiameter() * 
        moduleConfiguration.getDriveReduction();

        driveVelocityConversionFactor = drivePositionConversionFactor / 60.0;

        checkError(
            "Failed to set drive motor status frame period",
            driveMotor.getPosition().setUpdateFrequency(
                250,
                CAN_TIMEOUT_MS
            )
        );


        /*
         * Steer Motor Initialization
         */
        steerMotor = new CANSparkMax(steerMotorChannel, MotorType.kBrushless);
        checkError("Failed to restore steer motor factory defaults", steerMotor.restoreFactoryDefaults());

        // Reduce CAN status frame rates
        checkError(
            "Failed to set steer motor periodic status frame rate",
            steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100),
            steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20),
            steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20)
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
        steerEncoder.setInverted(SwerveConstants.KrakenSwerveModule.STEER_INVERTED);

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

        SparkPIDController steerController = steerMotor.getPIDController();
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
        return new SwerveModuleState(
            // Convert from ticks to meters per second using the predefined conversion factor
            driveMotor.getVelocity().getValueAsDouble() * driveVelocityConversionFactor,
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
            driveMotor.getPosition().getValueAsDouble() * drivePositionConversionFactor,
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
