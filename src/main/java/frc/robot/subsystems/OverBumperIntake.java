// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.Solenoid;

public class OverBumperIntake extends SubsystemBase {
  private final Talon overBumperIntake = new Talon(0);
  private Solenoid inSolonoid;
  private Solenoid outSolonoid;
 
  public OverBumperIntake(){
    inSolonoid = new Solenoid(null, 1);
    outSolonoid = new Solenoid(null, 2);
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
