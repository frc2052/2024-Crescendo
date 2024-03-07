// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;

public class GyroOffsetCommand extends Command {
  private double offset;
  /** Creates a new GyroOffsetCommand. */
  public GyroOffsetCommand(double offset) {
    this.offset = offset;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotState.getInstance().addNavXOffset(offset);
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
