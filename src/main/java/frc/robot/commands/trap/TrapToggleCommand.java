// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.trap;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TrapArmSubsystem;

public class TrapToggleCommand extends Command {
  private TrapArmSubsystem trapArm;

  /** Creates a new TrapReleaseCommand. */
  public TrapToggleCommand(TrapArmSubsystem trapArm) {
    this.trapArm = trapArm;
    // Use addRequirements() here to declare subsystem dependencies.
  addRequirements(trapArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(trapArm.getIsOpen()){
      trapArm.close();
    } else {
      trapArm.open();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
