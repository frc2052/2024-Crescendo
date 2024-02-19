// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private final CANSparkMax intakeMotor;

  public IntakeSubsystem () {    
    intakeMotor = new CANSparkMax(Constants.CAN.INTAKE_MOTOR_ID, MotorType.kBrushless);
  }

  public void intake() {
    intakeMotor.set(Constants.Intake.INTAKE_IN_SPEED_PCT);
  }

  public void outtake(){
    intakeMotor.set(Constants.Intake.INTAKE_OUT_SPEED_PCT);
  }

  public void stop() {
    intakeMotor.set(0);
  }

  public double getIntakeMotorSpeed() {
    return intakeMotor.get();
  }

  @Override
  public void periodic() {}
}
