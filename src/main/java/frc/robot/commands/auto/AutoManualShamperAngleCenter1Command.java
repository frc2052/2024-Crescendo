// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShamperSubsystem;

public class AutoManualShamperAngleCenter1Command extends Command {

  private ShamperSubsystem shamper;

  /** Creates a new AutoManualShamperAngle. */
  public AutoManualShamperAngleCenter1Command(ShamperSubsystem shamper) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shamper);
    this.shamper = shamper;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.shamper.setAngle(52);
    this.shamper.setShootSpeed(Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS * .9, Constants.Shamper.SHOOTER_MAX_VELOCITY_RPS * .9);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.shamper.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
