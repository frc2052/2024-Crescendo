// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;


import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class VerticalShooterSubsystem extends SubsystemBase {

  private final TalonFX lowerMotor;
  private final TalonFX upperMotor;

  public VerticalShooterSubsystem() {
    lowerMotor = new TalonFX(Constants.VerticalShooter.UPPER_SHOOTER_MOTOR_ID);
    upperMotor = new TalonFX(Constants.VerticalShooter.LOWER_SHOOTER_MOTOR_ID);
    lowerMotor.set(TalonFXControlMode.Velocity, Constants.VerticalShooter.LOWER_SHOOTER_IDLE_SPEED_TPS);
    upperMotor.set(TalonFXControlMode.Velocity, -Constants.VerticalShooter.UPPER_SHOOTER_IDLE_SPEED_TPS);
  }

  public void stop() {
    lowerMotor.set(TalonFXControlMode.Velocity, ShooterSpeeds.OFF.getLowerTPS());
    upperMotor.set(TalonFXControlMode.Velocity, ShooterSpeeds.OFF.getUpperTPS());
  }

  public void setShooterSpeed(ShooterSpeeds speeds) {
    lowerMotor.set(TalonFXControlMode.Velocity, speeds.getLowerTPS());
    upperMotor.set(TalonFXControlMode.Velocity, speeds.getUpperTPS());
  }

  public void setUpperShooterSpeed(double upperShooterMotorSpeedTPS) {
    upperMotor.set(TalonFXControlMode.Velocity, upperShooterMotorSpeedTPS);
  }

  public void setLowerShooterSpeed(double lowerShooterMotorSpeedTPS) {
    lowerMotor.set(TalonFXControlMode.Velocity, lowerShooterMotorSpeedTPS);
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