// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX intakeMotor;

  public IntakeSubsystem () {    
    intakeMotor = new TalonFX(Constants.CAN.INTAKE_MOTOR_ID);
  }

  public void intake() {
    intakeMotor.set(ControlMode.PercentOutput, Constants.Intake.INTAKE_IN_SPEED_PCT);
  }

  public void outtake(){
    intakeMotor.set(ControlMode.PercentOutput, -Constants.Intake.INTAKE_OUT_SPEED_PCT);
  }

  public void stop() {
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  public double getIntakeMotorSpeed() {
    return intakeMotor.getSelectedSensorVelocity();
  }

  @Override
  public void periodic() {}
}
