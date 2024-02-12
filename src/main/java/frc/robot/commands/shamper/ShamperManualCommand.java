// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shamper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShamperSubsystem;

public class ShamperManualCommand extends Command {
  private ShamperSubsystem shamper;
  private boolean goingUp;
  /** Creates a new ShamperManualCommand. */
  public ShamperManualCommand(ShamperSubsystem shamper, boolean goingUp) {
    this.shamper = shamper;
    this.goingUp = goingUp;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shamper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
        if (goingUp) {
      shamper.manualUp();
    } else {
      shamper.manualDown();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shamper.stopManual();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
