// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class UnderIntake1Side extends SubsystemBase {

  private final TalonFX topMotor;
  private final TalonFX bottomMotor;

  public UnderIntake1Side () {    
    topMotor = new TalonFX(Constants.UnderIntake1SideConstants.TOP_MOTOR_CHANNEL);
    bottomMotor = new TalonFX(Constants.UnderIntake1SideConstants.BOTTOM_MOTOR_CHANNEL);
  }

  public void intakeIn() {
    //reversed bottom motor
    topMotor.set(ControlMode.Velocity, Constants.UnderIntake1SideConstants.INTAKE_IN_SPEED_TPS);
    bottomMotor.set(ControlMode.Velocity, -Constants.UnderIntake1SideConstants.INTAKE_IN_SPEED_TPS);
  }

  public void intakeOut() {
    //reversed top motor
    topMotor.set(ControlMode.Velocity, -Constants.UnderIntake1SideConstants.INTAKE_OUT_SPEED_TPS);
    bottomMotor.set(ControlMode.Velocity, Constants.UnderIntake1SideConstants.INTAKE_OUT_SPEED_TPS);
  }
  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
