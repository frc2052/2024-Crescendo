// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class UnderIntake1SideSubsystem extends SubsystemBase {

  private final TalonFX upperMotor;
  private final TalonFX lowerMotor;

  public UnderIntake1SideSubsystem () {    
    upperMotor = new TalonFX(Constants.UnderIntake1SideConstants.UPPER_MOTOR_CHANNEL);
    lowerMotor = new TalonFX(Constants.UnderIntake1SideConstants.LOWER_MOTOR_CHANNEL);
  }

  public void intakeIn() {
    //reversed bottom motor
    upperMotor.set(ControlMode.Velocity, Constants.UnderIntake1SideConstants.INTAKE_IN_SPEED_TPS);
    lowerMotor.set(ControlMode.Velocity, -Constants.UnderIntake1SideConstants.INTAKE_IN_SPEED_TPS);
  }

  public void intakeOut() {
    //reversed top motor
    upperMotor.set(ControlMode.Velocity, -Constants.UnderIntake1SideConstants.INTAKE_OUT_SPEED_TPS);
    lowerMotor.set(ControlMode.Velocity, Constants.UnderIntake1SideConstants.INTAKE_OUT_SPEED_TPS);
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

  @Override
  public void simulationPeriodic() {}
}
