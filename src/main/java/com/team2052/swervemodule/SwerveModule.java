package com.team2052.swervemodule;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
   private final TalonFX driveMotor;
   private final CANSparkMax steerMotor;
   private final CANcoder canCoder;
   private String debugName;

   public SwerveModule(
        String debugName,
        int driveMotorChannel,
        int steerMotorChannel,
        int canCoderChannel,
        Rotation2d steerOffset
   ) {

    this.debugName = debugName;

        /*
         * CANCoder Initialization
         */
        CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
        canCoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        canCoderConfiguration.MagnetSensor.MagnetOffset = (steerOffset.getRadians() / (2 * Math.PI));
        canCoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        canCoder = new CANcoder(canCoderChannel);
        
        checkError(
            "Failed to configure CANCoder",
            canCoder.getConfigurator().apply(
                canCoderConfiguration,
                SwerveConstants.CAN_TIMEOUT_SECONDS
            )
        );

    /*
     * Drive Motor Initialization
     */

    driveMotor = new TalonFX(driveMotorChannel);
    // checkError("Failed to restore drive motor factory defaults", driveMotor.restoreFactoryDefaults());

    // checkError(
    //     "Failed to set drive motor periodic status frame rate",
    //     driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100),
    //     driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20),
    //     driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20)
    // );

    // checkError(
    //     "Failed to set drive motor idle mode",
    //     driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake)
    // );

    driveMotor.setNeutralMode(NeutralModeValue.Brake);

    driveMotor.setInverted(SwerveConstants.SwerveModule.DRIVE_INVERTED);

    // current limiting?

    VoltageConfigs driveVoltageConfig = new VoltageConfigs();
    driveVoltageConfig.withPeakForwardVoltage(SwerveConstants.MAX_VOLTAGE_VOLTS);
    driveVoltageConfig.withPeakReverseVoltage(SwerveConstants.MAX_VOLTAGE_VOLTS);

    checkError(
        "Failed to enable drive motor voltage compensation",
        driveMotor.getConfigurator().apply(
            driveVoltageConfig, 
            SwerveConstants.CAN_TIMEOUT_SECONDS)
    );

    CurrentLimitsConfigs driveCurrentLimitConfig = new CurrentLimitsConfigs();
    driveCurrentLimitConfig.withSupplyCurrentLimitEnable(true);
    driveCurrentLimitConfig.withStatorCurrentLimit(SwerveConstants.DRIVE_STALL_CURRENT_LIMIT_AMPS);
    driveCurrentLimitConfig.withSupplyCurrentLimit(SwerveConstants.DRIVE_FREE_CURRENT_LIMIT_AMPS);

    checkError(
        "Failed to set steer motor current limit",
        driveMotor.getConfigurator().apply(
            driveCurrentLimitConfig,
            SwerveConstants.CAN_TIMEOUT_SECONDS
        )
    );

    /*
     * Steer Motor Initialization
     */

    steerMotor = new CANSparkMax(steerMotorChannel, MotorType.kBrushless);
    checkError("Failed to restore drive motor factory defaults", steerMotor.restoreFactoryDefaults());

    checkError(
        "Failed to set drive motor periodic status frame rate",
        steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100),
        steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20),
        steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20)
    );

    checkError(
        "Failed to set drive motor idle mode",
        steerMotor.setIdleMode(IdleMode.kBrake)
    );

    steerMotor.setInverted(SwerveConstants.SwerveModule.STEER_INVERTED);

    checkError(
        "Failed to enable steer motor voltage compensation",
        steerMotor.enableVoltageCompensation(SwerveConstants.MAX_VOLTAGE_VOLTS)
    );

    checkError(
        "Failed to set steer motor current limit",
        steerMotor.setSmartCurrentLimit((int) SwerveConstants.STEER_CURRENT_LIMIT_AMPS)
    );

    // Drive Motor encoder initialization
    RelativeEncoder steerEncoder = steerMotor.getEncoder();
    
    checkError(
        "Failed to set drive motor encoder conversion factors",
        // Set the position conversion factor so the encoder will automatically convert ticks to radians
        steerEncoder.setPositionConversionFactor(SwerveConstants.SwerveModule.steerPositionConversionFactor),
        // Velocity of the encoder in radians per second
        steerEncoder.setVelocityConversionFactor(SwerveConstants.SwerveModule.steerPositionConversionFactor / 60.0)
    );

    checkError(
        "Failed to set steer motor encoder position",
        steerEncoder.setPosition(canCoder.getAbsolutePosition().getValueAsDouble() * (2 * Math.PI))
    );

    SparkPIDController steerController = steerMotor.getPIDController();
    checkError(
        "Failed to configure steer motor PID",
        steerController.setP(SwerveConstants.SwerveModule.STEER_MOTOR_P),
        steerController.setI(SwerveConstants.SwerveModule.STEER_MOTOR_I),
        steerController.setD(SwerveConstants.SwerveModule.STEER_MOTOR_D),
        steerController.setPositionPIDWrappingMinInput(-Math.PI),
        steerController.setPositionPIDWrappingMaxInput(Math.PI),
        steerController.setPositionPIDWrappingEnabled(true)
    );

    checkError(
        "Failed to set steer motor PID feedback device",
        steerController.setFeedbackDevice(steerEncoder)
    );
   }

    public SwerveModuleState getState() {
        // Both encoder values are automatically in units of meters per second and
        // radians because of the position and velocity conversion factors
        return new SwerveModuleState(
            driveMotor.getVelocity().getValueAsDouble(),
            new Rotation2d(
                (steerMotor.getEncoder().getPosition() % (2.0 * Math.PI))
            )
        );
    }

    public void setState(double velocityMetersPerSecond, Rotation2d steerAngle) {
        SwerveModuleState desiredState = new SwerveModuleState(velocityMetersPerSecond, steerAngle);
        // Reduce radians to 0 to 2pi range and simplify to nearest angle
        desiredState = SwerveModuleState.optimize(
            desiredState,
            getState().angle
        );

        // Set the motor to our desired velocity as a percentage of our max velocity
        driveMotor.set(desiredState.speedMetersPerSecond / getMaxVelocityMetersPerSecond());

        steerMotor.getPIDController().setReference(
            desiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition
        );

        SmartDashboard.putNumber(debugName + ": Desired Rotation", steerAngle.getDegrees());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveMotor.getRotorPosition().getValueAsDouble() * SwerveConstants.SwerveModule.drivePositionConversionFactor,
            new Rotation2d(
                steerMotor.getEncoder().getPosition()
            )
        );
    }

    public TalonFX getTalonFX(){
        return driveMotor;
    }

    public static double getMaxVelocityMetersPerSecond() {
        /*
         * The formula for calculating the theoretical maximum velocity is:
         * [Motor free speed (RPM)] / 60 * [Drive reduction] * [Wheel diameter (m)] * pi
         * This is a measure of how fast the robot should be able to drive in a straight line.
         */
        return SwerveConstants.SwerveModule.KRAKEN_ROUNDS_PER_MINUTE / 60 * SwerveConstants.SwerveModule.DRIVE_REDUCTION * 
            SwerveConstants.SwerveModule.WHEEL_DIAMETER_METERS * Math.PI;
    }

    public void debug() {
        SmartDashboard.putNumber(debugName + " Absolute CANcoder Degrees", canCoder.getAbsolutePosition().getValueAsDouble() * 360);
        SmartDashboard.putNumber(debugName + " Current CANcoder Degrees", canCoder.getPosition().getValueAsDouble() * 360);
        SmartDashboard.putNumber(debugName + " Current Steer Encoder Degrees", Math.toDegrees(steerMotor.getEncoder().getPosition()));
    }

    @SuppressWarnings("unchecked")
    protected <E> void checkError(String message, E... errors) {
        for (E error : errors) {
            if (error != REVLibError.kOk && error != StatusCode.OK) {
                DriverStation.reportError(
                    message + " on [" + debugName + "] module: " + error.toString(),
                    false
                );
            }
        }
    }
}  
