// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HorizontalShooterSubsystem extends SubsystemBase {

  private final TalonFX leftmotor;
  private final TalonFX rightmotor;
  
  public HorizontalShooterSubsystem() {
    leftmotor = new TalonFX(Constants.HorizontalShooter.LEFT_SHOOTER_MOTOR_ID);
    rightmotor = new TalonFX(Constants.HorizontalShooter.RIGHT_SHOOTER_MOTOR_ID);
    rightmotor.set(ControlMode.Velocity, Constants.HorizontalShooter.SHOOTER_DEFAULT_SPEED_TPS);
    leftmotor.set(ControlMode.Velocity, -Constants.HorizontalShooter.SHOOTER_DEFAULT_SPEED_TPS);
  }

  public void setLeftShooterSpeed(double leftMotorSpeedTPS ) {
    leftmotor.set(ControlMode.Velocity, leftMotorSpeedTPS);

  }

  public void setRightShooterSpeed(double rightMotorSpeedTPS ) {
    rightmotor.set(ControlMode.Velocity, rightMotorSpeedTPS);
    
  }

  public void stop() {
    leftmotor.set(ControlMode.Velocity, 0);
    rightmotor.set(ControlMode.Velocity, 0);
  }

  public void idle() {
    leftmotor.set(ControlMode.Velocity, -Constants.HorizontalShooter.SHOOTER_DEFAULT_SPEED_TPS);
    rightmotor.set(ControlMode.Velocity, Constants.HorizontalShooter.SHOOTER_DEFAULT_SPEED_TPS);
  }

  public double getRightShooterSpeed() {
    return rightmotor.getSelectedSensorVelocity();
  }

  public double getLeftShooterSpeed() {
    return leftmotor.getSelectedSensorVelocity();
  }

  @Override
  public void periodic() {}
}
