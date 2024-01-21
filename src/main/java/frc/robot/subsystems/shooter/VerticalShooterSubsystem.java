// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;


import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class VerticalShooterSubsystem extends SubsystemBase {
  /** Creates a new VerticalShooterSubsystem. */

  private final TalonFX lowerMotor;
  private final TalonFX upperMotor;

  public VerticalShooterSubsystem() {
    lowerMotor = new TalonFX(Constants.VerticalShooter.UPPER_SHOOTER_MOTOR_ID);
    upperMotor = new TalonFX(Constants.VerticalShooter.LOWER_SHOOTER_MOTOR_ID);

    upperMotor.setInverted(true);

    lowerMotor.set(TalonFXControlMode.Velocity, Constants.VerticalShooter.SHOOTER_IDLE_SPEED_TPS);
    upperMotor.set(TalonFXControlMode.Velocity, Constants.VerticalShooter.SHOOTER_IDLE_SPEED_TPS);
  }

  public void setUpperShooterSpeed(double upperShooterMotorSpeedTPS) {
    upperMotor.set(TalonFXControlMode.Velocity, upperShooterMotorSpeedTPS);
  }

  public void setLowerShooterSpeed(double lowerShooterMotorSpeedTPS) {
    lowerMotor.set(TalonFXControlMode.Velocity, lowerShooterMotorSpeedTPS);
  }

  public void stopShooterMotor() {
    lowerMotor.set(TalonFXControlMode.Velocity, 0);
    upperMotor.set(TalonFXControlMode.Velocity, 0);
  }

  public void returnToIdleSpeed() {
    lowerMotor.set(TalonFXControlMode.Velocity, Constants.VerticalShooter.SHOOTER_IDLE_SPEED_TPS);
    upperMotor.set(TalonFXControlMode.Velocity, Constants.VerticalShooter.SHOOTER_IDLE_SPEED_TPS);
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
}
;