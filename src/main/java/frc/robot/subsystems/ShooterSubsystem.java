// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX lowerMotor;
  private final TalonFX upperMotor;
  private final TalonFX rotationMotor;
  private final AnalogEncoder rotationEncoder;

  private ProfiledPIDController lowerMotorController;
  private ProfiledPIDController upperMotorController;
  private ProfiledPIDController rotationMotorController;

  public ShooterSubsystem() {

    lowerMotor = new TalonFX(Constants.VerticalShooter.UPPER_SHOOTER_MOTOR_ID);
    lowerMotorController = new ProfiledPIDController(
    Constants.VerticalShooter.LOWER_SHOOTER_KP,
    Constants.VerticalShooter.LOWER_SHOOTER_KI,
    Constants.VerticalShooter.LOWER_SHOOTER_KD, 
    new TrapezoidProfile.Constraints(
    Constants.VerticalShooter.LOWER_SHOOTER_MAX_VELOCITY,
    Constants.VerticalShooter.LOWER_SHOOTER_MAX_ACCELORATION));

    upperMotor = new TalonFX(Constants.VerticalShooter.LOWER_SHOOTER_MOTOR_ID);
    upperMotorController = new ProfiledPIDController(
    Constants.VerticalShooter.UPPER_SHOOTER_KP,
    Constants.VerticalShooter.UPPER_SHOOTER_KI,
    Constants.VerticalShooter.UPPER_SHOOTER_KD,      
    new TrapezoidProfile.Constraints(
    Constants.VerticalShooter.UPPER_SHOOTER_MAX_VELOCITY,
    Constants.VerticalShooter.UPPER_SHOOTER_MAX_ACCELORATION));

    rotationMotor = new TalonFX(Constants.VerticalShooter.ROTATION_SHOOTER_MOTOR_ID);
    rotationMotorController = new ProfiledPIDController(
    Constants.VerticalShooter.ROTATION_SHOOTER_KP,
    Constants.VerticalShooter.ROTATION_SHOOTER_KI,
    Constants.VerticalShooter.ROTATION_SHOOTER_KD,      
    new TrapezoidProfile.Constraints(
    Constants.VerticalShooter.ROTATION_SHOOTER_MAX_VELOCITY,
    Constants.VerticalShooter.ROTATION_SHOOTER_MAX_ACCELORATION));

    rotationEncoder = new AnalogEncoder(Constants.VerticalShooter.ROTATION_ENCODER_ID);

    lowerMotor.setInverted(Constants.VerticalShooter.LOWER_MOTOR_IS_INVERTED);
    upperMotor.setInverted(Constants.VerticalShooter.UPPER_MOTOR_IS_INVERTED);
    rotationMotor.setInverted(Constants.VerticalShooter.ROTATION_MOTOR_IS_INVERTED);
  }

  public void stop() {
    lowerMotor.set(TalonFXControlMode.Velocity, ShooterSpeeds.OFF.getLowerTPS());
    upperMotor.set(TalonFXControlMode.Velocity, ShooterSpeeds.OFF.getUpperTPS());
  }


  public void setSpeed(ShooterSpeeds speeds) {
    lowerMotor.set(TalonFXControlMode.Velocity, lowerMotorController.calculate(getLowerShooterSpeed(), speeds.getLowerTPS()));
    upperMotor.set(TalonFXControlMode.Velocity, upperMotorController.calculate(getUpperShooterSpeed(), speeds.getUpperTPS()));
  }

  public void setUpperShooterSpeed(double upperShooterMotorSpeedTPS) {
    upperMotor.set(TalonFXControlMode.Velocity, upperMotorController.calculate(upperShooterMotorSpeedTPS, upperShooterMotorSpeedTPS));
  }

  public void setLowerShooterSpeed(double lowerShooterMotorSpeedTPS) {
    lowerMotor.set(TalonFXControlMode.Velocity, lowerMotorController.calculate(lowerShooterMotorSpeedTPS, lowerShooterMotorSpeedTPS));
  }

  public double getUpperShooterSpeed() {
    return upperMotor.getSelectedSensorVelocity();
  }

  public double getLowerShooterSpeed() {
    return lowerMotor.getSelectedSensorVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public static enum ShooterSpeeds {
    OFF(0, 0),
    IDLING(Constants.VerticalShooter.LOWER_SHOOTER_IDLE_SPEED_TPS, Constants.VerticalShooter.UPPER_SHOOTER_IDLE_SPEED_TPS),
    SPEAKER(Constants.VerticalShooter.LOWER_SHOOTER_SPEAKER_SPEED_TPS, Constants.VerticalShooter.UPPER_SHOOTER_SPEAKER_SPEED_TPS),
    AMP(Constants.VerticalShooter.LOWER_SHOOTER_AMP_SPEED_TPS, Constants.VerticalShooter.UPPER_SHOOTER_AMP_SPEED_TPS);

    private final int lowerTicksPerSecond;
    private final int upperTicksPerSecond;

    private ShooterSpeeds(int lowerTicksPerSecond, int upperTicksPerSecond) {
      this.lowerTicksPerSecond = lowerTicksPerSecond;
      this.upperTicksPerSecond = upperTicksPerSecond;
    }

    public int getLowerTPS() {
      return lowerTicksPerSecond;
    }

    public int getUpperTPS() {
      return upperTicksPerSecond;
    }
  }
}