// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HorizontalShooterSubsystem extends SubsystemBase {

  private final TalonFX leftmotor;
  private final TalonFX rightmotor;
  public HorizontalShooterSubsystem() {
    leftmotor = new TalonFX(Constants.HorizontalShooterConstants.LEFT_SHOOTER_MOTOR_ID);
    rightmotor = new TalonFX(Constants.HorizontalShooterConstants.RIGHT_SHOOTER_MOTOR_ID);
    rightmotor.set(ControlMode.Velocity, Constants.HorizontalShooterConstants.SHOOTER_DEFAULT_SPEED_TPS);
    leftmotor.set(ControlMode.Velocity, -Constants.HorizontalShooterConstants.SHOOTER_DEFAULT_SPEED_TPS);
  }

  public void setLeftShooterSpeed(double LEFTMOTORSPEEDTPS ) {
    leftmotor.set(ControlMode.Velocity, LEFTMOTORSPEEDTPS);

  }
  public void setRightShooterSpeed(double RIGHTMOTORSPEEDTPS ) {
    rightmotor.set(ControlMode.Velocity, RIGHTMOTORSPEEDTPS);
    
  }
  public void stopShooterMotor() {
    leftmotor.set(ControlMode.Velocity, 0);
    rightmotor.set(ControlMode.Velocity, 0);
  }
  public void goBackToIdleSpeed() {
    leftmotor.set(ControlMode.Velocity, -Constants.HorizontalShooterConstants.SHOOTER_DEFAULT_SPEED_TPS);
    rightmotor.set(ControlMode.Velocity, Constants.HorizontalShooterConstants.SHOOTER_DEFAULT_SPEED_TPS);
  }

  public double getRightShooterSpeed() {
    return rightmotor.getSelectedSensorVelocity();
  }
  public double getLeftShooterSpeed() {
    return leftmotor.getSelectedSensorVelocity();
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
