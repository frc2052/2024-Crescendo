// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.util.RobotStateEstimator;

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
    //RobotStateEstimator.getInstance().resetOdometry(new Pose2d(1.33, 7.026501927468906, new Rotation2d(Units.degreesToRadians(-126.78843142290575))));
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
