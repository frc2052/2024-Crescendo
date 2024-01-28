// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX upperMotor;
  private final TalonFX lowerMotor;

  public IntakeSubsystem () {    
    upperMotor = new TalonFX(Constants.OneUnderBumperIntake.UPPER_MOTOR_CHANNEL);
    lowerMotor = new TalonFX(Constants.OneUnderBumperIntake.LOWER_MOTOR_CHANNEL);
  }

  public void setUpperMotorSpeed(double upperMotorSpeedTPS) {
    upperMotor.set(ControlMode.Velocity, upperMotorSpeedTPS);
  }

  public void setLowerMotorSpeed(double lowerMotorSpeedTPS) {
    lowerMotor.set(ControlMode.Velocity, lowerMotorSpeedTPS);
  }

  public void intakeIn() {
    //reversed bottom motor
    upperMotor.set(ControlMode.Velocity, Constants.OneUnderBumperIntake.INTAKE_IN_SPEED_TPS);
    lowerMotor.set(ControlMode.Velocity, -Constants.OneUnderBumperIntake.INTAKE_IN_SPEED_TPS);
  }

  public void intakeOut() {
    //reversed top motor
    upperMotor.set(ControlMode.Velocity, -Constants.OneUnderBumperIntake.INTAKE_OUT_SPEED_TPS);
    lowerMotor.set(ControlMode.Velocity, Constants.OneUnderBumperIntake.INTAKE_OUT_SPEED_TPS);
  }

  public void stop() {
    upperMotor.set(ControlMode.Velocity, 0);
    lowerMotor.set(ControlMode.Velocity, 0);
  }

  public double getUpperMotorSpeed() {
    return upperMotor.getSelectedSensorVelocity();
  }
  
  public double getLowerMotorSpeed() {
    return lowerMotor.getSelectedSensorVelocity();
  }

  @Override
  public void periodic() {}
}
