// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.Solenoid;

public class OverBumperIntakeSubsystem extends SubsystemBase {
  private final Talon overBumperIntake = new Talon(Constants.OverTheBumperIntake.INTAKE_MOTOR_ID);
  private Solenoid inSolonoid;
  private Solenoid outSolonoid;
 
  public OverBumperIntakeSubsystem(){
    inSolonoid = new Solenoid(null, Constants.OverTheBumperIntake.INSOLENOID_ID);
    outSolonoid = new Solenoid(null, Constants.OverTheBumperIntake.OUTSOLENOID_ID);
  }

  public void intake(){
    overBumperIntake.set(-1);
  }

  public void outtake(){
    overBumperIntake.set(1);
  }

  public void extend(){
    inSolonoid.set(true);
    outSolonoid.set(false);
  }

  public void retract(){
    inSolonoid.set(false);
    outSolonoid.set(true);
  }

}
