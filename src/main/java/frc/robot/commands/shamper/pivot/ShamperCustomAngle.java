// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shamper.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.util.io.Dashboard;

public class ShamperCustomAngle extends Command {
  /** Creates a new ShamperCustomSubShoot. */
  private ShamperSubsystem shamper;
  
  public ShamperCustomAngle(ShamperSubsystem shamper) {
    this.shamper = shamper;
    addRequirements(shamper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shamper.setAngle(Dashboard.getInstance().getManualAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
