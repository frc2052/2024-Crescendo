// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shamper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.subsystems.ShamperSubsystem.ShamperSpeed;

public class ShamperAmpCommand extends Command {
  private ShamperSubsystem shamper;
  private IndexerSubsystem indexer;
  /** Creates a new ShamperAmpCommand. */
  public ShamperAmpCommand(ShamperSubsystem shamper, IndexerSubsystem indexer) {
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
    shamper.setShootSpeed(ShamperSpeed.AMP_SCORE);

    if(shamper.shooterAtSpeed(ShamperSpeed.AMP_SCORE.getLower(), ShamperSpeed.AMP_SCORE.getUpper()) && shamper.isAtGoalAngle()) {
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
