// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {

  public int intakeInSpeed = 1;
  public int intakeOutSpeed = 1;

  public UnderIntake1Side () {    
    TalonFX topMotor = new TalonFX();
    TalonFX bottomMotor = new TalonFX();
  }

  public void intakeIn() {
    topMotor.set(ControlMode.PercentOutput, intakeInSpeed);
  }
  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
