// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shamper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.subsystems.ShamperSubsystem.ShamperSpeed;

public class ShamperSubCommand extends Command {
  private ShamperSubsystem shamper;
  private IndexerSubsystem indexer;
  /** Creates a new ShamperAmpCommand. */
  public ShamperSubCommand(ShamperSubsystem shamper, IndexerSubsystem indexer) {
    this.shamper = shamper;
    this. indexer = indexer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shamper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shamper.setShootSpeed(ShamperSpeed.SPEAKER_IDLE);
    shamper.setAngle(Constants.Shamper.Angle.SUB);
    if(shamper.shooterAtSpeed(ShamperSpeed.SPEAKER_IDLE.getLower(), ShamperSpeed.SPEAKER_IDLE.getUpper()) && shamper.isAtGoalAngle()) {
      indexer.indexAmp();
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shamper.stopShooter();
    indexer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
