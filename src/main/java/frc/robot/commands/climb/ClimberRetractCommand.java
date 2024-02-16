// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberRetractCommand extends Command {
  /** Creates a new RaiseClimberCommand. */
   
  private final ClimberSubsystem hook;

  public ClimberRetractCommand(ClimberSubsystem hook) {
      this.hook = hook;
      addRequirements(hook);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hook.retract(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hook.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
