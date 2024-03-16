// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shamper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.subsystems.ShamperSubsystem.ShamperSpeed;
import frc.robot.util.io.Dashboard;

public class ShamperDefaultCommand extends Command {
  private ShamperSubsystem shamper;
  /** Creates a new ShamperDefaultCommand. */
  public ShamperDefaultCommand(ShamperSubsystem shamper) {
    this.shamper = shamper;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shamper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Dashboard.getInstance().shouldIdle()) {
      switch (shamper.getCurrentIdleMode()) {
        case NO_IDLE:
          System.out.println("NO IDLE SELECTED BUT WANT TO IDLE");
          return;
        case SPEAKER_IDLE:
          shamper.setShootSpeed(ShamperSpeed.SPEAKER_IDLE);
        case AMP_IDLE:
          shamper.setShootSpeed(ShamperSpeed.AMP_IDLE);
      }
    }
    if(!RobotState.getInstance().getNoteDetected() && shamper.getShamperAngle() > Constants.Shamper.Angle.DEFAULT) {
      shamper.setAngle(Constants.Shamper.Angle.DEFAULT);
    }
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
