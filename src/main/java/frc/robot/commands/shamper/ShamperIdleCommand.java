// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shamper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.subsystems.ShamperSubsystem.ShamperSpeed;

public class ShamperIdleCommand extends Command {
  private ShamperSubsystem shamper;
  private ShamperSpeed idleSpeed;
  /** Creates a new ShamperIdleCommand. */
  public ShamperIdleCommand(ShamperSubsystem shamper, ShamperSpeed idleSpeed) {
    this.shamper = shamper;
    this.idleSpeed = idleSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shamper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shamper.setShootSpeed(idleSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shamper.setShootSpeed(ShamperSpeed.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
