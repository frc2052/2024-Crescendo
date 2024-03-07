// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shamper;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.subsystems.ShamperSubsystem.ShamperSpeed;

public class ShamperTrapCommand extends Command {
  
  private ShuffleboardTab tab = Shuffleboard.getTab("trap");
  private GenericEntry upperMotorSpeedPct =
  tab.add("Upper Motor Shoot Speed Manual", 0).getEntry();
  private GenericEntry lowerMotorSpeedPct =
  tab.add("Lower Motor Shoot Speed Manual", 0).getEntry();

  private ShamperSubsystem shamper;
  /** Creates a new ShamperAmpCommand. */
  public ShamperTrapCommand(ShamperSubsystem shamper) {
    this.shamper = shamper;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shamper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  public double getValue(GenericEntry value){
    double num = value.getDouble(0);
    if(num > 1){
      num = 1;
    } else if (num < -1){
      num = -1;
    }

    return num;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shamper.setShootSpeedPct(getValue(lowerMotorSpeedPct), (getValue(upperMotorSpeedPct)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shamper.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
