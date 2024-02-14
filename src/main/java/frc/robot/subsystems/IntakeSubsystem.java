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
    upperMotor = new TalonFX(Constants.Intake.UPPER_MOTOR_ID);
    lowerMotor = new TalonFX(Constants.Intake.LOWER_MOTOR_ID);
  }

  public void intake() {
    upperMotor.set(ControlMode.PercentOutput, Constants.Intake.INTAKE_IN_SPEED_PCT);
    lowerMotor.set(ControlMode.PercentOutput, -Constants.Intake.INTAKE_IN_SPEED_PCT);
  }

  public void outtake(){
    upperMotor.set(ControlMode.PercentOutput, -Constants.Intake.INTAKE_OUT_SPEED_PCT);
    lowerMotor.set(ControlMode.PercentOutput, Constants.Intake.INTAKE_OUT_SPEED_PCT);
  }

  public void stop() {
    upperMotor.set(ControlMode.PercentOutput, 0);
    lowerMotor.set(ControlMode.PercentOutput, 0);
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
