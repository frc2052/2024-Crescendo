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
import frc.robot.util.io.Dashboard.ShamperIdleMode;

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
    // if(RobotState.getInstance().getIsClimbing()) {
    //   return;
    // }

    if(Dashboard.getInstance().shouldIdle()) {
      if(RobotState.getInstance().getNoteHeldDetected()){
        switch (shamper.getCurrentIdleMode()) {
          case SPEAKER_IDLE:
            shamper.setShootSpeed(ShamperSpeed.SPEAKER_IDLE);
            break;
          case AMP_IDLE:
            shamper.setShootSpeed(ShamperSpeed.AMP_IDLE);
            break;
        }
      }
    } else {
      shamper.windDownShooter();
    }

    // if((!RobotState.getInstance().getNoteStagedDetected() && !RobotState.getInstance().getNoteHeldDetected()) &&
    //     shamper.getShamperAngle() > Constants.Shamper.Angle.DEFAULT
    //   ) {
    //   shamper.setAngle(Constants.Shamper.Angle.DEFAULT);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // shamper.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
